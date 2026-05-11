import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { networkInterfaces, hostname } from 'node:os';
import { join } from 'node:path';
import { execFileSync } from 'node:child_process';

const certDir = join(process.cwd(), 'certs');
const rootKeyPath = join(certDir, 'quest-dev-root-ca-key.pem');
const rootPemPath = join(certDir, 'quest-dev-root-ca.pem');
const rootCerPath = join(certDir, 'quest-dev-root-ca.cer');
const serverKeyPath = join(certDir, 'localhost-key.pem');
const serverPemPath = join(certDir, 'localhost.pem');
const serialPath = join(certDir, 'quest-dev-root-ca.srl');
const tempCsrPath = join(certDir, 'localhost.csr');
const tempLeafPath = join(certDir, 'localhost-leaf.pem');
const tempExtPath = join(certDir, 'localhost.ext');
const tempRootConfigPath = join(certDir, 'quest-dev-root-ca.cnf');
const tempServerConfigPath = join(certDir, 'localhost.cnf');

mkdirSync(certDir, { recursive: true });
assertOpenSsl();

const dnsNames = new Set(['localhost', hostname()]);
const ipAddresses = new Set(['127.0.0.1']);

for (const candidate of collectLanIpv4Addresses()) {
  ipAddresses.add(candidate);
}

const envHosts = splitList(process.env.CERT_DNS_NAMES);
const envIps = splitList(process.env.CERT_IPS);

for (const entry of envHosts) {
  dnsNames.add(entry);
}

for (const entry of envIps) {
  ipAddresses.add(entry);
}

ensureRootCa();
issueServerCertificate([...dnsNames], [...ipAddresses]);

console.log('[certs] generated development certificates');
console.log(`[certs] root CA pem : ${rootPemPath}`);
console.log(`[certs] root CA cer : ${rootCerPath}`);
console.log(`[certs] server cert : ${serverPemPath}`);
console.log(`[certs] server key  : ${serverKeyPath}`);
console.log(`[certs] SAN DNS     : ${[...dnsNames].join(', ')}`);
console.log(`[certs] SAN IP      : ${[...ipAddresses].join(', ')}`);

const lanUrls = [...ipAddresses]
  .filter((value) => value !== '127.0.0.1')
  .map((value) => `https://${value}:8787`);

if (lanUrls.length > 0) {
  console.log('[certs] Quest URLs  :');
  for (const url of lanUrls) {
    console.log(`  - ${url}`);
  }
}

console.log('[certs] next steps   : import quest-dev-root-ca.cer into Quest, then run `npm run serve`');

function assertOpenSsl() {
  try {
    execFileSync('openssl', ['version'], { stdio: 'ignore' });
  } catch {
    throw new Error('OpenSSL is required but was not found in PATH.');
  }
}

function ensureRootCa() {
  if (!existsSync(rootKeyPath) || !existsSync(rootPemPath)) {
    writeFileSync(
      tempRootConfigPath,
      [
        '[req]',
        'distinguished_name=req_distinguished_name',
        'prompt=no',
        'x509_extensions=v3_ca',
        '',
        '[req_distinguished_name]',
        'CN=Quest3 WebXR Dev Root CA',
        '',
        '[v3_ca]',
        'basicConstraints=critical,CA:TRUE,pathlen:0',
        'keyUsage=critical,keyCertSign,cRLSign',
        'subjectKeyIdentifier=hash'
      ].join('\n')
    );

    exec('openssl', ['genrsa', '-out', rootKeyPath, '2048']);
    exec('openssl', [
      'req',
      '-x509',
      '-new',
      '-key',
      rootKeyPath,
      '-sha256',
      '-days',
      '3650',
      '-out',
      rootPemPath,
      '-config',
      tempRootConfigPath
    ]);
  }

  exec('openssl', ['x509', '-in', rootPemPath, '-outform', 'der', '-out', rootCerPath]);
  safeRemove(tempRootConfigPath);
}

function issueServerCertificate(dnsEntries, ipEntries) {
  const extLines = [
    'authorityKeyIdentifier=keyid,issuer',
    'basicConstraints=critical,CA:FALSE',
    'keyUsage=critical,digitalSignature,keyEncipherment',
    'extendedKeyUsage=serverAuth',
    'subjectAltName=@alt_names',
    '',
    '[alt_names]'
  ];

  dnsEntries.forEach((value, index) => {
    extLines.push(`DNS.${index + 1}=${value}`);
  });

  ipEntries.forEach((value, index) => {
    extLines.push(`IP.${index + 1}=${value}`);
  });

  writeFileSync(tempExtPath, extLines.join('\n'));
  writeFileSync(
    tempServerConfigPath,
    [
      '[req]',
      'distinguished_name=req_distinguished_name',
      'prompt=no',
      '',
      '[req_distinguished_name]',
      `CN=${dnsEntries[0]}`
    ].join('\n')
  );

  exec('openssl', ['genrsa', '-out', serverKeyPath, '2048']);
  exec('openssl', [
    'req',
    '-new',
    '-key',
    serverKeyPath,
    '-out',
    tempCsrPath,
    '-config',
    tempServerConfigPath
  ]);
  exec('openssl', [
    'x509',
    '-req',
    '-in',
    tempCsrPath,
    '-CA',
    rootPemPath,
    '-CAkey',
    rootKeyPath,
    '-CAcreateserial',
    '-out',
    tempLeafPath,
    '-days',
    '825',
    '-sha256',
    '-extfile',
    tempExtPath
  ]);

  const leafPem = readFileSync(tempLeafPath, 'utf8').trim();
  const rootPem = readFileSync(rootPemPath, 'utf8').trim();
  writeFileSync(serverPemPath, `${leafPem}\n${rootPem}\n`);

  safeRemove(tempCsrPath);
  safeRemove(tempLeafPath);
  safeRemove(tempExtPath);
  safeRemove(tempServerConfigPath);
}

function collectLanIpv4Addresses() {
  const interfaces = networkInterfaces();
  const results = [];

  for (const entries of Object.values(interfaces)) {
    for (const entry of entries ?? []) {
      if (entry.family !== 'IPv4' || entry.internal) {
        continue;
      }

      if (isIgnoredAddress(entry.address)) {
        continue;
      }

      results.push(entry.address);
    }
  }

  return results.sort((left, right) => scoreIp(left) - scoreIp(right));
}

function isIgnoredAddress(value) {
  return (
    value.startsWith('169.254.') ||
    value.startsWith('198.18.') ||
    value.startsWith('198.19.')
  );
}

function scoreIp(value) {
  if (value.startsWith('192.168.')) {
    return 0;
  }

  if (value.startsWith('10.')) {
    return 1;
  }

  const octets = value.split('.').map(Number);
  if (octets[0] === 172 && octets[1] >= 16 && octets[1] <= 31) {
    return 2;
  }

  const hash = createHash('sha1').update(value).digest('hex');
  return 100 + Number.parseInt(hash.slice(0, 4), 16);
}

function splitList(value) {
  return (value ?? '')
    .split(',')
    .map((entry) => entry.trim())
    .filter(Boolean);
}

function exec(command, args) {
  execFileSync(command, args, {
    stdio: 'inherit'
  });
}

function safeRemove(path) {
  rmSync(path, { force: true });
}
