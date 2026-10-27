// Page context: "main" = export map, "job" = admin job overview, "shared" = public job link (no login).
const MODE = document.body.dataset.mode || 'main';
const SHARE_TOKEN = document.body.dataset.share || '';
const isJobPage = MODE !== 'main';
const isShared = MODE === 'shared';
// Job link on a phone: full-screen map with bottom sheets, and a pothole card instead of Leaflet popups.
const phoneQuery = window.matchMedia('(max-width: 850px)');
const usesCard = () => isShared && phoneQuery.matches;
// On phones, keep fitted areas clear of the stats card (top) and the list sheet (bottom).
const fitPadding = () => (usesCard() ? { paddingTopLeft: [16, 120], paddingBottomRight: [16, 90] } : {});

const map = L.map('map', { zoomControl: false, attributionControl: true }).setView([-33.958169, 18.464563], isJobPage ? 14 : 12);
L.control.zoom({ position: 'bottomright' }).addTo(map);
const baseLayer = L.tileLayer('https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png?key=cb1_2bdj_1_d1339a96dea1ce18cd688349', { attribution: '&copy; OpenStreetMap contributors &copy; CARTO', maxZoom: 20 }).addTo(map);
baseLayer.on('tileerror', () => { document.querySelector('#map')?.classList.add('tiles-unavailable'); });
const layer = L.markerClusterGroup({
  maxClusterRadius: 45,
  disableClusteringAtZoom: 17,
  showCoverageOnHover: false,
  spiderfyOnMaxZoom: true,
  iconCreateFunction: (cluster) => {
    const count = cluster.getChildCount();
    const size = count < 10 ? 34 : count < 50 ? 40 : 48;
    return L.divIcon({ html: `<span>${count}</span>`, className: 'pothole-cluster', iconSize: [size, size] });
  },
}).addTo(map);

const severityColors = { critical: '#e5262b', high: '#ff7a1a', medium: '#f5b800', low: '#16a085' };
const statusColors = { unassigned: '#e5262b', assigned: '#1fa84f' };
const repairColors = { unfilled: '#e5262b', filled: '#1fa84f', not_pothole: '#8a96a3' };
const COMPLETED_COLOR = '#b8c0c9';
const repairLabels = { unfilled: 'Unfilled', filled: 'Filled', not_pothole: 'Not a pothole' };
let allDetections = [];
const markers = new Map(); // detection id -> { detection, marker } for the potholes currently drawn
let selectedDetectionId = null;
let suburbModeOn = false;
let exporting = false;
let fittedToJob = false;

const titleCase = (value) => value.charAt(0).toUpperCase() + value.slice(1);
const suburbLabel = (name) => name.toLowerCase().replace(/(^|[\s\-(])(\w)/g, (_, start, char) => start + char.toUpperCase());
const suburbText = (detection) => detection.suburb ? suburbLabel(detection.suburb) : 'Unknown suburb';
const measurement = (detection) => `${(detection.width_mm * detection.length_mm / 1000000).toFixed(2)} m² (${detection.width_mm} x ${detection.length_mm} mm)`;
// Materials use the repair cut (sides rounded up to 0.25 m steps, worked out by the server), not the measured size.
const repairAreaM2 = (detection) => detection.repair_area_m2;
const repairLine = (detection) => `Repair cut ${(detection.repair_cut_width_mm / 1000).toFixed(2)} x ${(detection.repair_cut_length_mm / 1000).toFixed(2)} m · ${formatMass(asphaltKg(repairAreaM2(detection)))} asphalt`;
const checkedValue = (name) => document.querySelector(`input[name="${name}"]:checked`)?.value || '';
const viewMode = () => checkedValue('view-mode') || (isJobPage ? 'progress' : 'status');
const repairStatus = (detection) => repairLabels[detection.repair_status] ? detection.repair_status : 'unfilled';
function markerColor(detection) {
  if (viewMode() === 'status') return statusColors[detection.job_id ? 'assigned' : 'unassigned'];
  if (viewMode() === 'progress') return repairColors[repairStatus(detection)];
  if (repairStatus(detection) !== 'unfilled') return COMPLETED_COLOR; // severity only matters for what's left to repair
  return severityColors[detection.severity];
}
// Job pages describe a pothole by repair progress; the export map by which job (if any) it belongs to.
const jobLabel = (detection) => isJobPage ? repairLabels[repairStatus(detection)] : (detection.job_id ? `Job #${detection.job_id}` : 'Unassigned');

function setStatus(message) {
  const status = document.querySelector('#status');
  if (status) status.textContent = message;
}

// While selecting suburbs the map shows exactly what Export would take: every unassigned pothole, filters ignored.
function isShown(detection) {
  if (suburbModeOn) return !detection.job_id && repairStatus(detection) === 'unfilled';
  if (!isJobPage && repairStatus(detection) !== 'unfilled') return false; // filled / not-a-pothole stay visible on their job's map only
  const assignment = checkedValue('assignment-filter');
  const severity = checkedValue('severity-filter');
  if (assignment === 'unassigned' && detection.job_id) return false;
  if (assignment === 'assigned' && !detection.job_id) return false;
  return !severity || detection.severity === severity;
}

function renderMarkers() {
  layer.clearLayers();
  markers.clear();
  const newMarkers = [];
  allDetections.filter(isShown).forEach((detection) => {
    const marker = L.circleMarker([detection.latitude, detection.longitude], markerStyle(detection));
    if (usesCard()) {
      // Phones: no hover, and a tap opens the bottom pothole card rather than a small popup.
      marker.on('click', (event) => { L.DomEvent.stop(event); openPotholeCard(detection.id); });
    } else {
      const hoverPreview = `<div class="hover-preview"><img src="${esc(displayImage(detection))}" alt="Pothole preview"><strong>Pothole · ${esc(suburbText(detection))}</strong><span>${titleCase(esc(detection.severity))} · ${esc(jobLabel(detection))} · ${measurement(detection)}</span></div>`;
      marker.bindTooltip(hoverPreview, { direction: 'top', offset: [0, -8], opacity: 1, className: 'image-tooltip' });
      const popupWidth = Math.min(340, window.innerWidth - 70); // fit narrow phone screens
      marker.bindPopup(popupHtml(detection), { maxWidth: popupWidth, minWidth: popupWidth });
      marker.on('click', () => { selectedDetectionId = detection.id; renderVisiblePotholes(); });
    }
    newMarkers.push(marker);
    markers.set(detection.id, { detection, marker });
  });
  layer.addLayers(newMarkers); // one bulk add, so the clusters are only calculated once
  if (!markers.has(selectedDetectionId)) selectedDetectionId = null;
  if (!allDetections.length) setStatus(isJobPage ? 'This job has no potholes.' : 'No potholes yet.');
  else setStatus(markers.size ? '' : (suburbModeOn ? 'All potholes are already in jobs.' : 'No potholes match these filters.'));
  renderLegend();
  renderVisiblePotholes();
  renderSuburbPanel();
  renderJobSummary();
}

function markerStyle(detection) {
  const color = markerColor(detection);
  const selected = usesCard() && detection.id === selectedDetectionId;
  return { radius: usesCard() ? 12 : 9, color: selected ? '#13243a' : color, fillColor: color, fillOpacity: .85, weight: selected ? 4 : 2 };
}

function restyleMarkers() {
  markers.forEach(({ detection, marker }) => marker.setStyle(markerStyle(detection)));
}

// On job pages, once a pothole is marked filled / not a pothole, its submitted photo replaces the detection photo.
const showsRepairPhoto = (detection) => isJobPage && Boolean(detection.repair_photo_url);
const displayImage = (detection) => showsRepairPhoto(detection) ? detection.repair_photo_url : detection.image_url;
const photoCaption = (detection) => repairStatus(detection) === 'filled' ? 'Photo after repair' : 'Photo submitted by crew';
const mapsUrl = (detection) => `https://www.google.com/maps/search/?api=1&query=${detection.latitude},${detection.longitude}`;

// Crews can only act on unfilled potholes; once marked, the pothole is locked (admins can still override).
function crewActions(detection, className) {
  const status = repairStatus(detection);
  if (status !== 'unfilled') {
    return `<p class="crew-done">${status === 'filled' ? 'Already marked filled' : 'Already marked as not a pothole'}${detection.repair_updated_at ? ` on ${formatDate(detection.repair_updated_at)}` : ''}.</p>`;
  }
  return `<div class="${className}"><button class="mark-filled" type="button" data-id="${detection.id}">Mark filled</button><button class="mark-not-pothole" type="button" data-id="${detection.id}">Not a pothole</button></div>`;
}

function popupHtml(detection) {
  const status = repairStatus(detection);
  const updated = detection.repair_updated_at ? `<small>${new Date(detection.repair_updated_at).toLocaleString()}</small>` : '';
  const reason = detection.repair_reason_label ? `<span>${esc(detection.repair_reason_label)}</span>` : '';
  const statusLine = isJobPage ? `<div class="repair-line"><span class="repair-badge" style="background:${repairColors[status]}">${repairLabels[status]}</span>${reason}${updated}</div>` : '';
  const otherPhoto = showsRepairPhoto(detection)
    ? `<a class="popup-link" href="${esc(detection.image_url)}" target="_blank" rel="noopener">View original detection photo</a>` : '';
  const caption = showsRepairPhoto(detection) ? `<span class="photo-caption">${photoCaption(detection)}</span>` : '';
  const maps = isShared ? `<a class="popup-link" href="${mapsUrl(detection)}" target="_blank" rel="noopener">Open in Google Maps</a>` : '';
  const actions = isShared ? crewActions(detection, 'popup-actions') : '';
  const adminAction = isShared || status !== 'unfilled' ? '' : `<button class="mark-not-pothole footer-action" type="button" data-id="${detection.id}">Not a pothole</button>`;
  return `<div class="popup"><span class="severity ${detection.severity}">${titleCase(esc(detection.severity))}</span><h3>Pothole #${detection.id}</h3><p>${esc(suburbText(detection))} · ${esc(jobLabel(detection))}<br>${measurement(detection)}<br>${repairLine(detection)}</p>${statusLine}${caption}<img src="${esc(displayImage(detection))}" alt="Pothole photo"><div class="popup-links">${maps}${otherPhoto}</div>${actions}<div class="popup-footer"><small>Detected ${new Date(detection.captured_at).toLocaleString()}</small>${adminAction}</div></div>`;
}

// Phone job link: bottom card for the tapped pothole, with big navigate / mark buttons.
const potholeCard = document.querySelector('#pothole-card');

function cardHtml(detection) {
  const status = repairStatus(detection);
  const caption = showsRepairPhoto(detection) ? `<span class="photo-caption">${photoCaption(detection)}</span>` : '';
  const reason = detection.repair_reason_label ? `<p class="card-reason">${esc(detection.repair_reason_label)}</p>` : '';
  const updated = detection.repair_updated_at ? ` · updated ${formatDate(detection.repair_updated_at)}` : '';
  return `${caption}<img class="card-photo" src="${esc(displayImage(detection))}" alt="Pothole photo">
    <div class="card-badges"><span class="repair-badge" style="background:${repairColors[status]}">${repairLabels[status]}</span><span class="repair-badge" style="background:${severityColors[detection.severity]}">${titleCase(esc(detection.severity))} severity</span></div>
    <h3>${esc(suburbText(detection))}</h3>
    <p class="card-meta">Pothole #${detection.id} · ${measurement(detection)}<br>${repairLine(detection)}<br>Detected ${formatDate(detection.captured_at)}${updated}</p>${reason}
    <a class="card-navigate" href="${mapsUrl(detection)}" target="_blank" rel="noopener"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m3 11 18-8-8 18-2-8-8-2z"/></svg>Navigate in Google Maps</a>
    ${crewActions(detection, 'card-actions')}`;
}

function openPotholeCard(id, { pan = true } = {}) {
  const detection = allDetections.find((item) => item.id === id);
  if (!potholeCard || !detection) return;
  selectedDetectionId = id;
  document.querySelector('#pothole-card-body').innerHTML = cardHtml(detection);
  potholeCard.hidden = false;
  potholeCard.scrollTop = 0;
  document.body.classList.add('card-open');
  setListExpanded(false);
  restyleMarkers();
  if (pan) {
    // Zoom in if the pothole is hidden inside a cluster, then centre it in the part of the map the card doesn't cover.
    const marker = markers.get(id)?.marker;
    if (marker && layer.getVisibleParent(marker) !== marker) map.setView([detection.latitude, detection.longitude], 17, { animate: false });
    requestAnimationFrame(() => {
      const point = map.project([detection.latitude, detection.longitude]).add([0, potholeCard.offsetHeight / 2]);
      map.panTo(map.unproject(point), { animate: true });
    });
  }
}

function closePotholeCard() {
  if (!potholeCard || potholeCard.hidden) return;
  potholeCard.hidden = true;
  document.body.classList.remove('card-open');
  selectedDetectionId = null;
  restyleMarkers();
}

function setListExpanded(expanded) {
  const sheet = document.querySelector('.shared-page .detection-list');
  if (!sheet) return;
  sheet.classList.toggle('expanded', expanded);
  document.querySelector('#list-toggle')?.setAttribute('aria-expanded', String(expanded));
}

// "My location" (job link): a live blue dot so crews can see where they are relative to the potholes.
let locationWatch = null;
let locationDot = null;
let accuracyRing = null;

function stopLocating() {
  if (locationWatch !== null) navigator.geolocation.clearWatch(locationWatch);
  locationWatch = null;
  locationDot?.remove();
  accuracyRing?.remove();
  locationDot = null;
  accuracyRing = null;
  document.querySelector('#locate-me')?.setAttribute('aria-pressed', 'false');
}

function toggleLocating() {
  if (locationWatch !== null) return stopLocating();
  if (!navigator.geolocation || !window.isSecureContext) {
    return setStatus('Your location needs a secure (https) connection.');
  }
  let centred = false;
  document.querySelector('#locate-me')?.setAttribute('aria-pressed', 'true');
  setStatus('Finding your location...');
  locationWatch = navigator.geolocation.watchPosition((position) => {
    lastPosition = position; // reused when the crew submits a photo
    const latlng = [position.coords.latitude, position.coords.longitude];
    if (!locationDot) {
      accuracyRing = L.circle(latlng, { radius: position.coords.accuracy, color: '#1754fc', weight: 1, fillOpacity: .08, interactive: false }).addTo(map);
      locationDot = L.circleMarker(latlng, { radius: 7, color: '#ffffff', weight: 3, fillColor: '#1754fc', fillOpacity: 1, interactive: false }).addTo(map);
    } else {
      locationDot.setLatLng(latlng);
      accuracyRing.setLatLng(latlng).setRadius(position.coords.accuracy);
    }
    if (!centred) {
      centred = true;
      map.setView(latlng, Math.max(map.getZoom(), 16));
      setStatus('');
    }
  }, (error) => {
    setStatus(error.code === error.PERMISSION_DENIED ? 'Location permission was denied.' : 'Could not get your location.');
    stopLocating();
  }, { enableHighAccuracy: true, maximumAge: 10000, timeout: 20000 });
}

function renderLegend() {
  const legend = document.querySelector('#legend');
  if (!legend) return;
  const legends = {
    status: ['JOB STATUS', [['Unassigned', statusColors.unassigned], ['In a job', statusColors.assigned]]],
    progress: ['PROGRESS', Object.entries(repairLabels).map(([status, label]) => [label, repairColors[status]])],
    severity: ['SEVERITY', [...Object.entries(severityColors).map(([severity, color]) => [titleCase(severity), color]),
      ...(isJobPage ? [['Completed', COMPLETED_COLOR]] : [])]],
  };
  const [title, items] = legends[viewMode()] ?? legends.severity;
  legend.innerHTML = `<span class="eyebrow">${title}</span>${items.map(([label, color]) => `<div><i style="background:${color}"></i>${label}</div>`).join('')}`;
}

function renderVisiblePotholes() {
  const list = document.querySelector('#pothole-list');
  if (!list) return;
  const bounds = map.getBounds();
  // The job link's list is the crew's to-do list: every unfilled pothole in the job, wherever the map is.
  // Other pages list what's in view.
  const visible = [...markers.values()].filter(({ detection }) => (isShared
    ? repairStatus(detection) === 'unfilled'
    : bounds.contains([detection.latitude, detection.longitude])));
  const count = document.querySelector('#visible-count');
  if (count) count.textContent = visible.length;
  list.innerHTML = visible.length ? visible.map(({ detection }) => `<button class="list-item${selectedDetectionId === detection.id ? ' selected' : ''}" type="button" data-id="${detection.id}"><img class="list-image" src="${esc(displayImage(detection))}" alt="Pothole preview" onerror="this.classList.add('image-missing'); this.removeAttribute('src');"><span class="list-dot" style="background:${markerColor(detection)}"></span><span><strong>${titleCase(esc(detection.severity))} pothole</strong><small>${esc(suburbText(detection))} · ${esc(jobLabel(detection))}</small><small>${measurement(detection)}</small><small>${repairLine(detection)}</small></span></button>`).join('') : (isShared ? '<p class="list-empty">No potholes left to repair.</p>' : '<p class="list-empty">Pan or zoom the map to find potholes.</p>');
  list.querySelectorAll('.list-item').forEach((item) => item.addEventListener('click', () => selectDetection(Number(item.dataset.id))));
}

function jobProgress() {
  const counts = { unfilled: 0, filled: 0, not_pothole: 0 };
  let area = 0;
  let remainingArea = 0;
  allDetections.forEach((detection) => {
    const status = repairStatus(detection);
    counts[status] += 1;
    if (status === 'not_pothole') return;
    area += repairAreaM2(detection);
    if (status === 'unfilled') remainingArea += repairAreaM2(detection);
  });
  const repairable = counts.unfilled + counts.filled; // "not a pothole" doesn't count towards the job
  return { counts, area, remainingArea, repairable, percent: repairable ? Math.round(counts.filled / repairable * 100) : 0 };
}

function renderJobSummary() {
  if (!isJobPage) return;
  const { counts, area, remainingArea, repairable, percent } = jobProgress();
  const summary = document.querySelector('#job-summary-stats');
  if (summary) summary.textContent = `${counts.filled} of ${repairable} filled · ${area.toFixed(2)} m² cut · ${formatMass(asphaltKg(area))} asphalt (${formatMass(asphaltKg(remainingArea))} still needed)`;

  // Shared page: the left sidebar is the job's stats panel.
  const bar = document.querySelector('#job-progress-bar');
  if (bar) bar.style.width = `${percent}%`;
  const progressText = document.querySelector('#job-progress-text');
  if (progressText) {
    progressText.textContent = allDetections.length && !counts.unfilled
      ? 'Job complete: no potholes left to repair'
      : `${counts.filled} of ${plural(repairable, 'pothole')} filled (${percent}%)`;
  }
  const statList = document.querySelector('#job-stat-list');
  if (statList) {
    const rows = [
      ...Object.entries(repairLabels).map(([status, label]) => [`<i style="background:${repairColors[status]}"></i>${label}`, counts[status]]),
      ['Repair cut area', `${area.toFixed(2)} m²`],
      ['Asphalt (total)', formatMass(asphaltKg(area))],
      ['Asphalt (remaining)', formatMass(asphaltKg(remainingArea))],
    ];
    statList.innerHTML = rows.map(([label, value]) => `<div><dt>${label}</dt><dd>${value}</dd></div>`).join('');
  }
}

// Job link: a "Job complete" popup whenever the page opens on a finished job, and right after the last pothole is marked.
const jobIsComplete = () => allDetections.length > 0 && jobProgress().counts.unfilled === 0;

function showJobComplete() {
  const modal = document.querySelector('#complete-modal');
  if (!modal) return;
  const { counts } = jobProgress();
  const parts = [`${plural(counts.filled, 'pothole')} filled`];
  if (counts.not_pothole) parts.push(`${counts.not_pothole} marked as not a pothole`);
  document.querySelector('#complete-text').textContent = `Every pothole in Job #${JOB.id} has been dealt with: ${parts.join(', ')}. Thank you!`;
  modal.hidden = false;
}

function renderJobDetails() {
  const created = document.querySelector('#job-created');
  if (created) created.textContent = `Created ${new Date(JOB.created_at).toLocaleDateString(undefined, { day: 'numeric', month: 'long', year: 'numeric' })}`;
}

// Shared page: mark a pothole filled or "not a pothole" (with a reason). Both need a proof photo.
let actionTarget = null; // { id, status } of the pothole being updated
let previewUrl = null;
const sheet = document.querySelector('#action-sheet');

function showActionError(message) {
  const error = document.querySelector('#action-error');
  error.textContent = message;
  error.hidden = !message;
}

// Crews on the job link must attach a photo; dashboard admins only choose a reason.
const needsPhoto = isShared;

function openActionSheet(id, status) {
  const detection = allDetections.find((item) => item.id === id);
  if (!sheet || !detection) return;
  actionTarget = { id, status };
  document.querySelector('#action-form').reset();
  document.querySelector('#action-title').textContent = status === 'filled' ? 'Mark pothole filled' : 'Not a pothole';
  const instructions = status === 'filled' ? 'Take a photo of the filled pothole.' : (needsPhoto ? 'Choose what it is and take a photo of it.' : 'Choose what it is.');
  document.querySelector('#action-subtitle').textContent = `${titleCase(detection.severity)} pothole in ${suburbText(detection)}. ${instructions}`;
  document.querySelector('#action-detection-image').src = detection.image_url; // the original detection, to judge what it is
  document.querySelector('#reason-field').hidden = status !== 'not_pothole';
  document.querySelector('#photo-field').hidden = !needsPhoto;
  document.querySelector('#photo-picker-text').textContent = 'Take or choose a photo';
  document.querySelector('#action-preview').hidden = true;
  showActionError('');
  sheet.hidden = false;
  map.closePopup();
}

function closeActionSheet() {
  if (sheet) sheet.hidden = true;
  actionTarget = null;
  if (previewUrl) URL.revokeObjectURL(previewUrl);
  previewUrl = null;
}

// Best-effort phone position for the status history; null when unavailable (no https, denied, or too slow).
let lastPosition = null;
function currentPosition() {
  if (!navigator.geolocation || !window.isSecureContext) return Promise.resolve(null);
  if (lastPosition && Date.now() - lastPosition.timestamp < 30000) return Promise.resolve(lastPosition);
  return new Promise((resolve) => {
    navigator.geolocation.getCurrentPosition(resolve, () => resolve(null), { enableHighAccuracy: true, maximumAge: 30000, timeout: 8000 });
  });
}

// Shrink phone photos (often 3-8 MB) to a 1600 px JPEG before upload; fall back to the original file.
async function preparePhoto(file) {
  try {
    const bitmap = await createImageBitmap(file);
    const scale = Math.min(1, 1600 / Math.max(bitmap.width, bitmap.height));
    const canvas = document.createElement('canvas');
    canvas.width = Math.round(bitmap.width * scale);
    canvas.height = Math.round(bitmap.height * scale);
    canvas.getContext('2d').drawImage(bitmap, 0, 0, canvas.width, canvas.height);
    const blob = await new Promise((resolve) => canvas.toBlob(resolve, 'image/jpeg', 0.85));
    if (blob) return blob;
  } catch { /* browser can't decode it here; let the server validate the original */ }
  return file;
}

async function submitAction(event) {
  event.preventDefault();
  if (!actionTarget) return;
  const { id, status } = actionTarget;
  const reason = document.querySelector('#action-reason').value;
  const file = document.querySelector('#action-photo').files[0];
  if (status === 'not_pothole' && !reason) return showActionError('Choose what it is.');
  if (needsPhoto && !file) return showActionError('A photo is required.');
  const submit = document.querySelector('#action-submit');
  submit.disabled = true;
  submit.textContent = needsPhoto ? 'Getting location...' : 'Saving...';
  showActionError('');
  try {
    let response;
    if (needsPhoto) {
      const body = new FormData();
      body.append('status', status);
      if (status === 'not_pothole') body.append('reason', reason);
      const position = await currentPosition();
      if (position) {
        body.append('crew_latitude', position.coords.latitude);
        body.append('crew_longitude', position.coords.longitude);
        body.append('crew_accuracy', position.coords.accuracy);
      }
      submit.textContent = 'Uploading...';
      body.append('photo', await preparePhoto(file), 'photo.jpg');
      response = await fetch(`/api/v1/shared/${encodeURIComponent(SHARE_TOKEN)}/detections/${id}/status`, { method: 'POST', body });
    } else {
      response = await fetch(`/api/v1/detections/${id}/status`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ status, reason }) });
    }
    const result = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(result.error || 'Unable to save. Please try again.');
    const wasComplete = jobIsComplete();
    allDetections = allDetections.map((detection) => detection.id === result.id ? result : detection);
    if (isShared && !wasComplete && jobIsComplete()) setTimeout(showJobComplete, 400); // that was the last one
    closeActionSheet();
    renderMarkers();
    if (usesCard() && selectedDetectionId === result.id) openPotholeCard(result.id, { pan: false }); // show the new status
    if (!isShared) loadJobs(); // job counts / completion may have changed
    setStatus(status === 'filled' ? 'Pothole marked as filled.' : 'Marked as not a pothole.');
  } catch (error) {
    showActionError(error.message);
  } finally {
    submit.disabled = false;
    submit.textContent = 'Submit';
  }
}

document.querySelector('#action-form')?.addEventListener('submit', submitAction);
document.querySelector('#action-cancel')?.addEventListener('click', closeActionSheet);
sheet?.addEventListener('click', (event) => { if (event.target === sheet) closeActionSheet(); });
document.addEventListener('keydown', (event) => { if (event.key === 'Escape' && sheet && !sheet.hidden) closeActionSheet(); });
document.querySelector('#action-photo')?.addEventListener('change', (event) => {
  const file = event.target.files[0];
  const preview = document.querySelector('#action-preview');
  if (previewUrl) URL.revokeObjectURL(previewUrl);
  previewUrl = file ? URL.createObjectURL(file) : null;
  preview.hidden = !file;
  if (file) preview.src = previewUrl;
  document.querySelector('#photo-picker-text').textContent = file ? 'Change photo' : 'Take or choose a photo';
  showActionError('');
});

// Admin job page: copy the public job link, creating it first for older jobs that don't have one yet.
async function copyJobLink() {
  const button = document.querySelector('#copy-job-link');
  const label = button?.querySelector('span');
  const input = document.querySelector('#job-link');
  if (!button || !label) return;
  button.disabled = true;
  try {
    if (!JOB.share_url) {
      const response = await fetch(`/api/v1/jobs/${JOB.id}/share`, { method: 'POST' });
      const result = await response.json().catch(() => ({}));
      if (!response.ok) throw new Error(result.error || 'Unable to create job link');
      JOB.share_url = result.share_url;
    }
    if (input) { input.value = JOB.share_url; input.hidden = false; }
    try {
      await navigator.clipboard.writeText(JOB.share_url);
      label.textContent = 'Link copied';
    } catch {
      // Clipboard access needs HTTPS or localhost; fall back to a selected field the user can copy from.
      input?.select();
      label.textContent = 'Copy the link below';
    }
    setTimeout(() => { label.textContent = 'Copy job link'; }, 2500);
  } catch (error) {
    setStatus(error.message);
  } finally {
    button.disabled = false;
  }
}

function selectDetection(id) {
  const entry = markers.get(id);
  if (!entry) return;
  if (usesCard()) return openPotholeCard(id);
  selectedDetectionId = id;
  document.querySelectorAll('.list-item').forEach((item) => item.classList.toggle('selected', Number(item.dataset.id) === id));
  layer.zoomToShowLayer(entry.marker, () => entry.marker.openPopup()); // opens its cluster first if needed
}

function clearSelection() {
  if (usesCard()) { closePotholeCard(); setListExpanded(false); return; }
  selectedDetectionId = null;
  map.closePopup();
  renderVisiblePotholes();
}

async function loadDetections() {
  setStatus('Loading potholes...');
  try {
    const query = isShared ? `?share=${encodeURIComponent(SHARE_TOKEN)}` : (isJobPage ? `?job_id=${JOB.id}` : '');
    const response = await fetch(`/api/v1/detections${query}`);
    if (!response.ok) throw new Error('Unable to load potholes');
    allDetections = (await response.json()).detections;
    renderMarkers();
    if (isShared && jobIsComplete()) showJobComplete();
    // Fall back to fitting the potholes when the job has no suburb outlines to fit to.
    if (isJobPage && !fittedToJob && !JOB.suburbs?.length && allDetections.length) {
      fittedToJob = true;
      map.fitBounds(L.latLngBounds(allDetections.map((d) => [d.latitude, d.longitude])).pad(0.15), { maxZoom: 17, ...fitPadding() });
    }
  } catch (error) { setStatus(error.message); }
}

// Suburb selection: polygons live in their own pane below the overlay pane (z 400) so pothole markers stay on top and clickable.
map.createPane('suburbs').style.zIndex = 350;
const suburbStyles = {
  base: { color: '#1f6f9f', weight: 1, opacity: .7, fillColor: '#4c9bc2', fillOpacity: .06 },
  hover: { weight: 2.5, opacity: 1, fillOpacity: .18 },
  selectedHover: { weight: 3, opacity: 1, fillOpacity: .45 },
  selected: { color: '#1754fc', weight: 2, opacity: 1, fillColor: '#1754fc', fillOpacity: .35 },
  jobOutline: { color: '#6fa8ff', weight: 2.5, opacity: .95, dashArray: '6 5', fillOpacity: 0 },
};
const selectedSuburbs = new Map();
let suburbLayer = null;
let suburbDataRequest = null;

function suburbStyle(feature) {
  return selectedSuburbs.has(feature.properties.id) ? { ...suburbStyles.base, ...suburbStyles.selected } : suburbStyles.base;
}

function loadSuburbData() {
  suburbDataRequest ??= fetch('/static/data/suburbs.geojson').then((response) => {
    if (!response.ok) throw new Error('Unable to load suburb boundaries');
    return response.json();
  }).catch((error) => { suburbDataRequest = null; throw error; });
  return suburbDataRequest;
}

function buildSuburbLayer(data) {
  return L.geoJSON(data, {
    pane: 'suburbs',
    style: suburbStyle,
    onEachFeature: (feature, polygon) => {
      polygon.bindTooltip(esc(suburbLabel(feature.properties.name)), { sticky: true, direction: 'top', offset: [0, -6], className: 'suburb-tooltip' });
      polygon.on('mouseover', () => polygon.setStyle(selectedSuburbs.has(feature.properties.id) ? suburbStyles.selectedHover : suburbStyles.hover));
      polygon.on('mouseout', () => polygon.setStyle(suburbStyle(feature)));
      polygon.on('click', () => toggleSuburb(feature.properties.id, feature.properties.name));
    },
  });
}

// Job page: outline the suburbs the job was exported from and zoom to that whole area.
async function showJobSuburbs() {
  const names = new Set(JOB?.suburbs ?? []);
  if (!names.size) return;
  try {
    const data = await loadSuburbData();
    const outline = L.geoJSON({ type: 'FeatureCollection', features: data.features.filter((feature) => names.has(feature.properties.name)) }, {
      pane: 'suburbs',
      style: suburbStyles.jobOutline,
      interactive: false,
    }).addTo(map);
    if (outline.getLayers().length) {
      fittedToJob = true;
      map.fitBounds(outline.getBounds().pad(0.05), fitPadding());
    }
  } catch (error) { setStatus(error.message); }
}

function refreshSuburbStyles() {
  suburbLayer?.eachLayer((polygon) => polygon.setStyle(suburbStyle(polygon.feature)));
}

function toggleSuburb(id, name) {
  if (selectedSuburbs.has(id)) selectedSuburbs.delete(id);
  else selectedSuburbs.set(id, name);
  refreshSuburbStyles();
  renderSuburbPanel();
}

// Counts only unassigned potholes, regardless of the map filters, because that is what Export takes.
function suburbPotholeStats() {
  const stats = new Map();
  allDetections.forEach((detection) => {
    if (detection.job_id || !detection.suburb || repairStatus(detection) !== 'unfilled') return;
    const entry = stats.get(detection.suburb) ?? { count: 0, area: 0 };
    entry.count += 1;
    entry.area += repairAreaM2(detection);
    stats.set(detection.suburb, entry);
  });
  return stats;
}

function renderSuburbPanel() {
  const list = document.querySelector('#suburb-list');
  if (!list) return;
  const total = selectedSuburbs.size;
  const setText = (selector, value) => { const el = document.querySelector(selector); if (el) el.textContent = value; };
  setText('#suburb-count', `${plural(total, 'suburb')} selected`);
  document.querySelector('#suburb-clear').disabled = total === 0;
  document.querySelector('#suburb-summary').hidden = total === 0;
  const stats = suburbPotholeStats();
  const sorted = [...selectedSuburbs].sort(([, a], [, b]) => a.localeCompare(b));
  const totals = { count: 0, area: 0 };
  list.innerHTML = total ? sorted.map(([id, name]) => {
    const { count: potholes, area } = stats.get(name) ?? { count: 0, area: 0 };
    totals.count += potholes;
    totals.area += area;
    const detail = potholes ? `${plural(potholes, 'pothole')} · ${formatMass(asphaltKg(area))} asphalt` : 'No unassigned potholes';
    return `<li><span class="suburb-info"><strong>${esc(suburbLabel(name))}</strong><small>${detail}</small></span><button class="suburb-remove" type="button" data-id="${esc(id)}" aria-label="Remove ${esc(suburbLabel(name))}">&times;</button></li>`;
  }).join('') : '<li class="suburb-empty">Tap a suburb on the map to select it.</li>';
  setText('#suburb-total-potholes', totals.count);
  setText('#suburb-total-area', `${totals.area.toFixed(2)} m²`);
  setText('#suburb-total-material', formatMass(asphaltKg(totals.area)));
  const exportButton = document.querySelector('#suburb-export');
  exportButton.disabled = exporting || totals.count === 0;
  exportButton.textContent = exporting ? 'Creating job...' : (totals.count ? `Create Job with ${plural(totals.count, 'pothole')}` : 'Create Job');
}

async function exportSelection() {
  const suburbs = [...new Set(selectedSuburbs.values())];
  if (!suburbs.length || exporting) return;
  exporting = true;
  renderSuburbPanel();
  try {
    const response = await fetch('/api/v1/jobs', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ suburbs }) });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(result.error || 'Unable to create job');
    selectedSuburbs.clear();
    refreshSuburbStyles();
    await Promise.all([loadDetections(), loadJobs()]);
    setSuburbMode(false);
    showJobToast(result);
  } catch (error) {
    setStatus(error.message);
  } finally {
    exporting = false;
    renderSuburbPanel();
  }
}

let toastTimer = null;
function showJobToast(job) {
  const toast = document.querySelector('#job-toast');
  if (!toast) return;
  toast.innerHTML = `<span>Job #${job.id} created with ${plural(job.pothole_count, 'pothole')}.</span><a href="${esc(job.url)}">Open job</a>`;
  toast.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.hidden = true; }, 10000);
}

async function setSuburbMode(on) {
  const toggle = document.querySelector('#suburb-toggle');
  const panel = document.querySelector('#suburb-panel');
  suburbModeOn = on;
  toggle?.setAttribute('aria-pressed', String(on));
  toggle?.classList.toggle('active', on);
  const label = toggle?.querySelector('span');
  if (label) label.textContent = on ? 'Cancel' : 'Create a job';
  document.body.classList.toggle('creating-job', on);
  const banner = document.querySelector('#create-banner');
  if (banner) banner.hidden = !on;
  if (on) document.querySelector('#job-toast')?.setAttribute('hidden', '');
  if (!on && selectedSuburbs.size) { selectedSuburbs.clear(); refreshSuburbStyles(); renderSuburbPanel(); } // Cancel discards the selection
  if (panel) panel.hidden = !on;
  document.querySelectorAll('.filter-section.lockable input').forEach((input) => { input.disabled = on; });
  const note = document.querySelector('#filter-note');
  if (note) note.hidden = !on;
  renderMarkers();
  if (!on) { if (suburbLayer) map.removeLayer(suburbLayer); return; }
  if (!suburbLayer) {
    if (toggle) toggle.disabled = true;
    setStatus('Loading suburbs...');
    try {
      suburbLayer = buildSuburbLayer(await loadSuburbData());
      if (document.querySelector('#status')?.textContent === 'Loading suburbs...') setStatus('');
    } catch (error) {
      setStatus(error.message);
      setSuburbMode(false);
      return;
    } finally {
      if (toggle) toggle.disabled = false;
    }
  }
  if (suburbModeOn) suburbLayer.addTo(map);
}

document.querySelector('#suburb-toggle')?.addEventListener('click', () => setSuburbMode(!suburbModeOn));
document.querySelector('#suburb-clear')?.addEventListener('click', () => { selectedSuburbs.clear(); refreshSuburbStyles(); renderSuburbPanel(); });
document.querySelector('#suburb-export')?.addEventListener('click', exportSelection);
document.querySelector('#suburb-list')?.addEventListener('click', (event) => {
  const button = event.target.closest('.suburb-remove');
  if (!button) return;
  selectedSuburbs.delete(Number(button.dataset.id));
  refreshSuburbStyles();
  renderSuburbPanel();
});
document.querySelector('#suburb-collapse')?.addEventListener('click', (event) => {
  const panel = document.querySelector('#suburb-panel');
  const collapsed = panel?.classList.toggle('collapsed');
  event.currentTarget.setAttribute('aria-expanded', String(!collapsed));
});

document.querySelectorAll('input[name="view-mode"], input[name="assignment-filter"], input[name="severity-filter"]').forEach((input) => input.addEventListener('change', renderMarkers));
// Delegated so it covers both map popups and the phone pothole card.
document.addEventListener('click', (event) => {
  const filled = event.target.closest('.mark-filled');
  if (filled) openActionSheet(Number(filled.dataset.id), 'filled');
  const notPothole = event.target.closest('.mark-not-pothole');
  if (notPothole) openActionSheet(Number(notPothole.dataset.id), 'not_pothole');
});
map.on('click', clearSelection);
document.querySelector('#pothole-card-close')?.addEventListener('click', closePotholeCard);
const completeModal = document.querySelector('#complete-modal');
document.querySelector('#complete-close')?.addEventListener('click', () => { completeModal.hidden = true; });
completeModal?.addEventListener('click', (event) => { if (event.target === completeModal) completeModal.hidden = true; });
document.querySelector('#stats-toggle')?.addEventListener('click', (event) => {
  const expanded = document.querySelector('.job-sidebar').classList.toggle('expanded');
  event.currentTarget.setAttribute('aria-expanded', String(expanded));
});
document.querySelector('#filters-toggle')?.addEventListener('click', (event) => {
  const open = document.body.classList.toggle('filters-open');
  event.currentTarget.setAttribute('aria-pressed', String(open));
});
document.querySelector('#locate-me')?.addEventListener('click', toggleLocating);
const listToggle = document.querySelector('.shared-page #list-toggle');
listToggle?.addEventListener('click', () => { if (usesCard()) setListExpanded(!document.querySelector('.detection-list').classList.contains('expanded')); });
listToggle?.addEventListener('keydown', (event) => {
  if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); listToggle.click(); }
});
// Rotating a tablet (or resizing) across the phone breakpoint swaps between popups and the card.
phoneQuery.addEventListener('change', () => {
  closePotholeCard();
  setListExpanded(false);
  map.closePopup();
  renderMarkers();
  setTimeout(() => map.invalidateSize(), 0);
});
map.on('moveend zoomend', renderVisiblePotholes);
document.querySelector('#copy-job-link')?.addEventListener('click', copyJobLink);
document.querySelector('#job-link')?.addEventListener('focus', (event) => event.target.select());
renderLegend();
loadDetections();
if (isJobPage) { renderJobDetails(); showJobSuburbs(); }
