const ICON_WIDTH = 30;
const ICON_HEIGHT = 30;
const ICON_ROW_BYTES = Math.ceil(ICON_WIDTH / 8);
const MAX_ITEMS = 15;

const state = {
  path: "include/tile_menu_config.h",
  items: [],
  selectedIndex: 0,
  tool: "draw",
  drawing: false,
  lastCell: null,
  importSourceImage: null,
  importSourceName: "",
};

const els = {
  pathInput: document.getElementById("pathInput"),
  loadButton: document.getElementById("loadButton"),
  saveButton: document.getElementById("saveButton"),
  statusText: document.getElementById("statusText"),
  slotList: document.getElementById("slotList"),
  moveUpButton: document.getElementById("moveUpButton"),
  moveDownButton: document.getElementById("moveDownButton"),
  slotMeta: document.getElementById("slotMeta"),
  titleInput: document.getElementById("titleInput"),
  commandInput: document.getElementById("commandInput"),
  visibleState: document.getElementById("visibleState"),
  commandHex: document.getElementById("commandHex"),
  iconCanvas: document.getElementById("iconCanvas"),
  sourceCanvas: document.getElementById("sourceCanvas"),
  previewCanvas: document.getElementById("previewCanvas"),
  bytesOutput: document.getElementById("bytesOutput"),
  importButton: document.getElementById("importButton"),
  imageFileInput: document.getElementById("imageFileInput"),
  thresholdRange: document.getElementById("thresholdRange"),
  thresholdValue: document.getElementById("thresholdValue"),
  importInvertCheckbox: document.getElementById("importInvertCheckbox"),
  importInfoText: document.getElementById("importInfoText"),
  clearButton: document.getElementById("clearButton"),
  invertButton: document.getElementById("invertButton"),
  fillButton: document.getElementById("fillButton"),
  flipHButton: document.getElementById("flipHButton"),
  flipVButton: document.getElementById("flipVButton"),
  toolButtons: Array.from(document.querySelectorAll("[data-tool]")),
};

const iconCtx = els.iconCanvas.getContext("2d");
const sourceCtx = els.sourceCanvas.getContext("2d");
const previewCtx = els.previewCanvas.getContext("2d");

function createBlankPixels() {
  return Array.from({ length: ICON_HEIGHT }, () => Array(ICON_WIDTH).fill(0));
}

function normalizeItem(rawItem, index) {
  const pixels = Array.isArray(rawItem?.pixels)
    ? rawItem.pixels.map((row) => row.map((cell) => (cell ? 1 : 0)))
    : createBlankPixels();
  return {
    index,
    title: rawItem?.title ?? "",
    command: Number(rawItem?.command ?? 0) & 0xff,
    pixels,
  };
}

function isVisible(item) {
  return item.title.trim() !== "" && item.command !== 0;
}

function currentItem() {
  return state.items[state.selectedIndex];
}

function clearImportSource() {
  state.importSourceImage = null;
  state.importSourceName = "";
}

function updateItemIndexes() {
  state.items.forEach((item, index) => {
    item.index = index;
  });
}

function setStatus(text, kind = "idle") {
  els.statusText.textContent = text;
  els.statusText.className = `status ${kind}`;
}

function commandHex(command) {
  return `0x${(Number(command) & 0xff)
    .toString(16)
    .toUpperCase()
    .padStart(2, "0")}`;
}

function refreshImportControls() {
  els.thresholdValue.textContent = els.thresholdRange.value;
  els.importInfoText.textContent = state.importSourceName
    ? `源图: ${state.importSourceName}`
    : "未导入源图片";
  els.moveUpButton.disabled = state.selectedIndex <= 0;
  els.moveDownButton.disabled = state.selectedIndex >= state.items.length - 1;
}

function renderSlotList() {
  els.slotList.innerHTML = "";
  state.items.forEach((item, index) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `slot-button${index === state.selectedIndex ? " active" : ""}`;
    button.innerHTML = `
      <span class="slot-index">${String(index + 1).padStart(2, "0")}</span>
      <span class="slot-title">${item.title.trim() || "(empty)"}</span>
      <span class="slot-meta">
        ${commandHex(item.command)}
        <span class="${isVisible(item) ? "visible-tag" : "hidden-tag"}">
          ${isVisible(item) ? "显示" : "隐藏"}
        </span>
      </span>
    `;
    button.addEventListener("click", () => {
      state.selectedIndex = index;
      clearImportSource();
      renderAll();
    });
    els.slotList.appendChild(button);
  });
}

function syncFormFromState() {
  const item = currentItem();
  if (!item) {
    return;
  }
  els.slotMeta.textContent = `槽位 ${String(item.index + 1).padStart(2, "0")}`;
  els.titleInput.value = item.title;
  els.commandInput.value = item.command;
  refreshFormDerived();
}

function refreshFormDerived() {
  const item = currentItem();
  if (!item) {
    return;
  }
  els.visibleState.textContent = isVisible(item) ? "显示" : "隐藏";
  els.commandHex.textContent = commandHex(item.command);
}

function packPixels(pixels) {
  const bytes = Array(ICON_HEIGHT * ICON_ROW_BYTES).fill(0);
  for (let y = 0; y < ICON_HEIGHT; y += 1) {
    for (let x = 0; x < ICON_WIDTH; x += 1) {
      if (pixels[y][x]) {
        bytes[y * ICON_ROW_BYTES + Math.floor(x / 8)] |= 1 << (x % 8);
      }
    }
  }
  return bytes;
}

function updateBytesOutput() {
  const item = currentItem();
  if (!item) {
    els.bytesOutput.value = "";
    return;
  }

  const bytes = packPixels(item.pixels);
  const lines = [];
  for (let i = 0; i < bytes.length; i += 16) {
    lines.push(
      bytes
        .slice(i, i + 16)
        .map((value) => `0x${value.toString(16).toUpperCase().padStart(2, "0")}`)
        .join(", "),
    );
  }
  els.bytesOutput.value = lines.join("\n");
}

function renderEditorCanvas() {
  const item = currentItem();
  if (!item) {
    return;
  }

  const cellSize = els.iconCanvas.width / ICON_WIDTH;

  iconCtx.clearRect(0, 0, els.iconCanvas.width, els.iconCanvas.height);
  iconCtx.fillStyle = "#f8fbf2";
  iconCtx.fillRect(0, 0, els.iconCanvas.width, els.iconCanvas.height);

  for (let y = 0; y < ICON_HEIGHT; y += 1) {
    for (let x = 0; x < ICON_WIDTH; x += 1) {
      if (item.pixels[y][x]) {
        iconCtx.fillStyle = "#101010";
        iconCtx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
      }
      iconCtx.strokeStyle = "rgba(23, 32, 27, 0.12)";
      iconCtx.strokeRect(x * cellSize, y * cellSize, cellSize, cellSize);
    }
  }
}

function rasterizeImportImage(image) {
  const canvas = document.createElement("canvas");
  canvas.width = ICON_WIDTH;
  canvas.height = ICON_HEIGHT;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const srcWidth = image.naturalWidth || image.width;
  const srcHeight = image.naturalHeight || image.height;
  const scale = Math.max(ICON_WIDTH / srcWidth, ICON_HEIGHT / srcHeight);
  const drawWidth = srcWidth * scale;
  const drawHeight = srcHeight * scale;
  const dx = (ICON_WIDTH - drawWidth) / 2;
  const dy = (ICON_HEIGHT - drawHeight) / 2;

  ctx.drawImage(image, dx, dy, drawWidth, drawHeight);
  return canvas;
}

function renderSourcePreview() {
  sourceCtx.clearRect(0, 0, els.sourceCanvas.width, els.sourceCanvas.height);
  sourceCtx.fillStyle = "#ffffff";
  sourceCtx.fillRect(0, 0, els.sourceCanvas.width, els.sourceCanvas.height);

  if (!state.importSourceImage) {
    sourceCtx.strokeStyle = "rgba(23, 32, 27, 0.16)";
    sourceCtx.strokeRect(
      0.5,
      0.5,
      els.sourceCanvas.width - 1,
      els.sourceCanvas.height - 1,
    );
    sourceCtx.fillStyle = "#5f6d63";
    sourceCtx.font = "12px Segoe UI";
    sourceCtx.textAlign = "center";
    sourceCtx.textBaseline = "middle";
    sourceCtx.fillText("No Image", els.sourceCanvas.width / 2, els.sourceCanvas.height / 2);
    return;
  }

  const rasterCanvas = rasterizeImportImage(state.importSourceImage);
  sourceCtx.imageSmoothingEnabled = false;
  sourceCtx.drawImage(rasterCanvas, 0, 0, els.sourceCanvas.width, els.sourceCanvas.height);
}

function renderPreviewCanvas() {
  const item = currentItem();
  if (!item) {
    return;
  }

  const scale = els.previewCanvas.width / ICON_WIDTH;

  previewCtx.clearRect(0, 0, els.previewCanvas.width, els.previewCanvas.height);
  previewCtx.fillStyle = "#ffffff";
  previewCtx.fillRect(0, 0, els.previewCanvas.width, els.previewCanvas.height);

  previewCtx.fillStyle = "#050505";
  for (let y = 0; y < ICON_HEIGHT; y += 1) {
    for (let x = 0; x < ICON_WIDTH; x += 1) {
      if (item.pixels[y][x]) {
        previewCtx.fillRect(x * scale, y * scale, scale, scale);
      }
    }
  }
}

function renderToolButtons() {
  els.toolButtons.forEach((button) => {
    button.classList.toggle("active", button.dataset.tool === state.tool);
  });
}

function renderAll() {
  renderSlotList();
  syncFormFromState();
  refreshImportControls();
  renderToolButtons();
  renderEditorCanvas();
  renderSourcePreview();
  renderPreviewCanvas();
  updateBytesOutput();
}

function readCanvasCell(event) {
  const rect = els.iconCanvas.getBoundingClientRect();
  const scaleX = els.iconCanvas.width / rect.width;
  const scaleY = els.iconCanvas.height / rect.height;
  const x = Math.floor(
    ((event.clientX - rect.left) * scaleX) / (els.iconCanvas.width / ICON_WIDTH),
  );
  const y = Math.floor(
    ((event.clientY - rect.top) * scaleY) / (els.iconCanvas.height / ICON_HEIGHT),
  );
  if (x < 0 || x >= ICON_WIDTH || y < 0 || y >= ICON_HEIGHT) {
    return null;
  }
  return { x, y };
}

function applyToolAt(cell) {
  if (!cell) {
    return;
  }
  const item = currentItem();
  if (!item) {
    return;
  }
  if (state.lastCell && state.lastCell.x === cell.x && state.lastCell.y === cell.y) {
    return;
  }
  item.pixels[cell.y][cell.x] = state.tool === "draw" ? 1 : 0;
  state.lastCell = cell;
  renderAll();
}

function handleCanvasPointerDown(event) {
  state.drawing = true;
  state.lastCell = null;
  applyToolAt(readCanvasCell(event));
}

function handleCanvasPointerMove(event) {
  if (!state.drawing) {
    return;
  }
  applyToolAt(readCanvasCell(event));
}

function handleCanvasPointerUp() {
  state.drawing = false;
  state.lastCell = null;
}

function mutatePixels(mutator) {
  const item = currentItem();
  if (!item) {
    return;
  }
  item.pixels = mutator(item.pixels);
  renderAll();
}

function thresholdImportedImage() {
  if (!state.importSourceImage) {
    return;
  }

  const rasterCanvas = rasterizeImportImage(state.importSourceImage);
  const rasterCtx = rasterCanvas.getContext("2d");
  const { data } = rasterCtx.getImageData(0, 0, ICON_WIDTH, ICON_HEIGHT);
  const threshold = Number.parseInt(els.thresholdRange.value, 10) || 128;
  const invert = els.importInvertCheckbox.checked;
  const pixels = createBlankPixels();

  for (let y = 0; y < ICON_HEIGHT; y += 1) {
    for (let x = 0; x < ICON_WIDTH; x += 1) {
      const offset = (y * ICON_WIDTH + x) * 4;
      const gray =
        data[offset] * 0.299 +
        data[offset + 1] * 0.587 +
        data[offset + 2] * 0.114;
      const darkPixel = gray < threshold ? 1 : 0;
      pixels[y][x] = invert ? (darkPixel ? 0 : 1) : darkPixel;
    }
  }

  const item = currentItem();
  if (!item) {
    return;
  }
  item.pixels = pixels;
  renderAll();
}

function loadImageFromFile(file) {
  return new Promise((resolve, reject) => {
    const url = URL.createObjectURL(file);
    const image = new Image();
    image.onload = () => {
      URL.revokeObjectURL(url);
      resolve(image);
    };
    image.onerror = () => {
      URL.revokeObjectURL(url);
      reject(new Error("图片加载失败"));
    };
    image.src = url;
  });
}

async function importImageFile(file) {
  if (!file) {
    return;
  }
  const image = await loadImageFromFile(file);
  state.importSourceImage = image;
  state.importSourceName = file.name;
  thresholdImportedImage();
  setStatus(`已导入并二值化 ${file.name}`, "ok");
}

function moveSelectedItem(delta) {
  const from = state.selectedIndex;
  const to = from + delta;
  if (to < 0 || to >= state.items.length) {
    return;
  }

  const [item] = state.items.splice(from, 1);
  state.items.splice(to, 0, item);
  updateItemIndexes();
  state.selectedIndex = to;
  renderAll();
  setStatus(`已调整顺序到槽位 ${String(to + 1).padStart(2, "0")}`, "ok");
}

function loadFromResponse(payload) {
  state.path = payload.path || els.pathInput.value.trim();
  els.pathInput.value = state.path;
  state.items = Array.from({ length: MAX_ITEMS }, (_, index) =>
    normalizeItem(payload.items?.[index], index),
  );
  state.selectedIndex = Math.min(state.selectedIndex, state.items.length - 1);
  clearImportSource();
  renderAll();
}

async function loadConfig() {
  const path = els.pathInput.value.trim() || "include/tile_menu_config.h";
  setStatus("加载中...", "idle");
  const response = await fetch(`/api/config?path=${encodeURIComponent(path)}`);
  const payload = await response.json();
  if (!payload.ok) {
    throw new Error(payload.error || "load failed");
  }
  loadFromResponse(payload);
  setStatus(`已加载 ${payload.path}`, "ok");
}

async function saveConfig() {
  const payload = {
    path: els.pathInput.value.trim() || "include/tile_menu_config.h",
    items: state.items.map((item) => ({
      title: item.title,
      command: Number(item.command) & 0xff,
      pixels: item.pixels,
    })),
  };
  setStatus("保存中...", "idle");
  const response = await fetch("/api/config", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!result.ok) {
    throw new Error(result.error || "save failed");
  }
  loadFromResponse(result);
  setStatus(`已保存 ${result.path}`, "ok");
}

function bindEvents() {
  els.loadButton.addEventListener("click", () => {
    loadConfig().catch((error) => setStatus(error.message, "error"));
  });

  els.saveButton.addEventListener("click", () => {
    saveConfig().catch((error) => setStatus(error.message, "error"));
  });

  els.moveUpButton.addEventListener("click", () => {
    moveSelectedItem(-1);
  });

  els.moveDownButton.addEventListener("click", () => {
    moveSelectedItem(1);
  });

  els.titleInput.addEventListener("input", (event) => {
    const item = currentItem();
    if (!item) {
      return;
    }
    item.title = event.target.value;
    renderSlotList();
    refreshFormDerived();
    refreshImportControls();
  });

  els.commandInput.addEventListener("input", (event) => {
    const item = currentItem();
    if (!item) {
      return;
    }
    const value = Number.parseInt(event.target.value, 10);
    item.command = Number.isFinite(value) ? Math.max(0, Math.min(255, value)) : 0;
    renderSlotList();
    refreshFormDerived();
    refreshImportControls();
  });

  els.toolButtons.forEach((button) => {
    button.addEventListener("click", () => {
      state.tool = button.dataset.tool;
      renderToolButtons();
    });
  });

  els.importButton.addEventListener("click", () => {
    els.imageFileInput.click();
  });

  els.imageFileInput.addEventListener("change", (event) => {
    const [file] = event.target.files || [];
    importImageFile(file).catch((error) => setStatus(error.message, "error"));
    event.target.value = "";
  });

  els.thresholdRange.addEventListener("input", () => {
    refreshImportControls();
    if (state.importSourceImage) {
      thresholdImportedImage();
    }
  });

  els.importInvertCheckbox.addEventListener("change", () => {
    if (state.importSourceImage) {
      thresholdImportedImage();
    } else {
      refreshImportControls();
    }
  });

  els.clearButton.addEventListener("click", () => {
    mutatePixels(() => createBlankPixels());
  });

  els.fillButton.addEventListener("click", () => {
    mutatePixels(() =>
      Array.from({ length: ICON_HEIGHT }, () => Array(ICON_WIDTH).fill(1)),
    );
  });

  els.invertButton.addEventListener("click", () => {
    mutatePixels((pixels) => pixels.map((row) => row.map((value) => (value ? 0 : 1))));
  });

  els.flipHButton.addEventListener("click", () => {
    mutatePixels((pixels) => pixels.map((row) => [...row].reverse()));
  });

  els.flipVButton.addEventListener("click", () => {
    mutatePixels((pixels) => [...pixels].reverse().map((row) => [...row]));
  });

  els.iconCanvas.addEventListener("pointerdown", handleCanvasPointerDown);
  els.iconCanvas.addEventListener("pointermove", handleCanvasPointerMove);
  els.iconCanvas.addEventListener("pointerup", handleCanvasPointerUp);
  els.iconCanvas.addEventListener("pointerleave", handleCanvasPointerUp);
  els.iconCanvas.addEventListener("contextmenu", (event) => event.preventDefault());

  window.addEventListener("pointerup", handleCanvasPointerUp);
}

function bootstrap() {
  const query = new URLSearchParams(window.location.search);
  const requestedPath = query.get("path");
  if (requestedPath) {
    els.pathInput.value = requestedPath;
  }

  state.items = Array.from({ length: MAX_ITEMS }, (_, index) => normalizeItem(null, index));

  bindEvents();
  renderAll();
  loadConfig().catch((error) => setStatus(error.message, "error"));
}

bootstrap();
