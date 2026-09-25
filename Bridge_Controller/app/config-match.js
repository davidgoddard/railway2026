'use strict';

function sameCameraConfiguration(draft, saved) {
  if (!draft || !saved || draft.cells?.length !== saved.cells?.length) return false;
  const settings = ['resolution', 'brightness', 'contrast', 'saturation', 'vflip', 'hmirror'];
  if (settings.some(key => draft.settings?.[key] !== saved.settings?.[key])) return false;
  const fields = ['id', 'group', 'x', 'y', 'radius', 'shape', 'floor', 'threshold', 'enter', 'clear'];
  const savedCells = new Map(saved.cells.map(cell => [cell.id, cell]));
  if (savedCells.size !== saved.cells.length) return false;
  for (const cell of draft.cells) {
    const other = savedCells.get(cell.id);
    if (!other || fields.some(key => cell[key] !== other[key])) return false;
  }
  const outputIds = new Set(draft.cells.map(cell => cell.group || cell.id));
  for (const id of outputIds) {
    if ((draft.topics?.[id] || String(id)) !== (saved.topics?.[id] || String(id))) return false;
  }
  return true;
}

function countCameraChanges(draft, saved) {
  if (!draft) return 0;
  if (!saved) return (draft.cells?.length || 0) + 1;
  let count = 0;
  const settings = ['resolution', 'brightness', 'contrast', 'saturation', 'vflip', 'hmirror'];
  if (settings.some(key => draft.settings?.[key] !== saved.settings?.[key])) count++;
  const fields = ['group', 'x', 'y', 'radius', 'shape', 'floor', 'threshold', 'enter', 'clear'];
  const savedCells = new Map((saved.cells || []).map(cell => [cell.id, cell]));
  for (const cell of draft.cells || []) {
    const previous = savedCells.get(cell.id);
    if (!previous || fields.some(key => cell[key] !== previous[key])) count++;
    savedCells.delete(cell.id);
  }
  count += savedCells.size;
  const outputIds = new Set([...(draft.cells || []).map(cell => cell.group || cell.id), ...(saved.cells || []).map(cell => cell.group || cell.id)]);
  for (const id of outputIds) if ((draft.topics?.[id] || String(id)) !== (saved.topics?.[id] || String(id))) count++;
  return count;
}

function alignConfigurationToFrame(draft, frame) {
  if (!draft || !frame) return false;
  const dimensions = [[320, 240], [640, 480], [800, 600], [1024, 768]];
  const resolution = dimensions.findIndex(([width, height]) => width === frame.width && height === frame.height);
  if (resolution < 0) return false;
  draft.settings.resolution = resolution;
  return true;
}
function frameMatchesConfiguration(config, frame) {
  const dimensions = [[320, 240], [640, 480], [800, 600], [1024, 768]];
  const expected = dimensions[config?.settings?.resolution];
  return !!expected && expected[0] === frame?.width && expected[1] === frame?.height;
}

if (typeof module === 'object' && module.exports) module.exports = { sameCameraConfiguration, countCameraChanges, alignConfigurationToFrame, frameMatchesConfiguration };
else Object.assign(window, { sameCameraConfiguration, countCameraChanges, alignConfigurationToFrame, frameMatchesConfiguration });
