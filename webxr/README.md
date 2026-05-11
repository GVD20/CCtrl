# Quest 3 WebXR Controller 6DoF Demo

Three.js-based WebXR page for Quest 3 controller pose capture, local XYZ axis visualization, and real-time WebSocket streaming to a minimal Node receiver.

## Features

- Quest Browser `immersive-vr` demo scene with controller-local `X/Y/Z` arrows
- Per-controller floating debug panel showing position, quaternion, trigger, squeeze, and button state
- Pose streaming over WebSocket with a minimal receiver endpoint
- Desktop-safe preview mode that keeps the page usable when WebXR hardware is unavailable

## Scripts

- `python WebXR_link.py`: interactive one-stop CLI for USB XR-UART bridge, gnirehtet, certs, and WebXR service lifecycle
- `npm run certs:generate`: create a local root CA plus a server cert for `localhost` and your LAN IPs
- `npm run dev`: start the Vite frontend on `http://localhost:5173`
- `npm run server`: start the WebSocket receiver on `http://localhost:8787`
- `npm run build`: type-check and build the frontend
- `npm run serve`: serve the built `dist/` folder and `/ws` from the same Node server

## WebXR Link CLI

`webxr/WebXR_link.py` extracts the WebXR server + UART relay path from `tools/preview_monitor.py` into a standalone command-line entrypoint.

It handles:

1. startup cleanup for leftover `WebXR_link.py`, `preview_monitor.py`, `gnirehtet`, and WebXR server processes
2. USB serial scanning and selection, then `@XR XR_ON` at `2000000` baud
3. background `adb` + `gnirehtet run` for an already-connected Android device
4. first-launch auto `npm run build`
5. WebXR service start/restart with live status display
6. certificate check / optional generation / restart-to-enable-HTTPS

Typical usage:

```powershell
python .\webxr\WebXR_link.py
```

Optional arguments:

- `--serial-port COM6`
- `--android-serial 1WMHH...`
- `--baud 2000000`
- `--http-port 8787`

Cross-platform notes:

- Windows: the script prefers bundled `webxr/platform-tools/adb.exe` and `gnirehtet.exe`.
- Arch Linux: the script expects `adb`, `gnirehtet`, `node`, `npm`, and `openssl` in `PATH`.
- On both platforms, `pyserial` is required for the USB XR-UART bridge.

## Config

`index.html` exposes:

```html
<script>
  window.__WEBXR_CONFIG__ = {
    WS_URL: "",
    SEND_HZ: 60
  };
</script>
```

- `WS_URL`: explicit WebSocket endpoint. Leave empty to auto-resolve.
- `SEND_HZ`: pose-frame send rate. Defaults to `60`.

Auto-resolution behavior:

- HTTPS page -> `wss://<host>/ws`
- HTTP page -> `ws://localhost:8787/ws`

## Quest testing

For desktop development:

1. Run `npm run server`
2. Run `npm run dev`
3. Open the Vite page in a desktop browser for non-XR preview

For Quest same-origin hosting:

1. Run `npm run build`
2. Run `npm run certs:generate`
3. Import `certs/quest-dev-root-ca.cer` into Quest so the browser trusts your local CA
4. Run `npm run serve`
5. Open the served HTTPS LAN URL in Quest Browser and enter VR

### Why `ERR_SSL_PROTOCOL_ERROR` happens

If you open `https://<your-ip>:8787` but `npm run serve` started without cert files, the Node server falls back to plain HTTP on port `8787`. Quest then tries to speak TLS to a non-TLS socket and shows `ERR_SSL_PROTOCOL_ERROR`.

The fix is:

1. Generate certs with `npm run certs:generate`
2. Restart the server with `npm run serve`
3. Import `certs/quest-dev-root-ca.cer` on Quest so the certificate is trusted

### Generated cert files

`npm run certs:generate` creates:

- `certs/quest-dev-root-ca.pem`: local development root CA in PEM format
- `certs/quest-dev-root-ca.cer`: same root CA exported in DER format for Quest import
- `certs/localhost.pem`: server certificate chain for Node HTTPS
- `certs/localhost-key.pem`: private key for the server

By default the generated leaf certificate includes:

- `DNS: localhost`
- `DNS: <your Windows hostname>`
- `IP: 127.0.0.1`
- all detected LAN IPv4 addresses such as `192.168.x.x`

You can add extra names with:

```powershell
$env:CERT_DNS_NAMES="my-hostname"
$env:CERT_IPS="192.168.1.250"
npm run certs:generate
```

### Importing the root CA on Quest

1. Copy `certs/quest-dev-root-ca.cer` to the Quest headset.
2. In Quest system settings, open the certificate / security credentials import flow.
3. Import that `.cer` file as a CA certificate.
4. Reopen Quest Browser after import.

The exact settings label can vary by Quest OS version, but the file you need is always `quest-dev-root-ca.cer`.

Useful endpoints:

- `/health`: basic health summary
- `/status`: active session cache and latest frame info
