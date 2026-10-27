// Status history "Find a pothole": search by pothole ID or address, or click the map to drop a pin,
// then list every pothole near the pin (any status), nearest first, with its latest update and photos.
const finderMap = L.map('finder-map').setView([-33.958169, 18.464563], 12);
L.tileLayer('https://basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png?key=cb1_2bdj_1_d1339a96dea1ce18cd688349', { attribution: '&copy; OpenStreetMap contributors &copy; CARTO', maxZoom: 20 }).addTo(finderMap);
const resultLayer = L.layerGroup().addTo(finderMap);
const finderColors = { unfilled: '#e5262b', filled: '#1fa84f', not_pothole: '#8a96a3' };
const finderLabels = { unfilled: 'Unfilled', filled: 'Filled', not_pothole: 'Not a pothole' };
const resultMarkers = new Map();
let pin = null;
let radiusRing = null;
let lastSearch = null; // { lat, lon } or { id }, re-run when the radius changes

const titleWord = (value) => value.charAt(0).toUpperCase() + value.slice(1);
const suburbName = (name) => (name ? name.toLowerCase().replace(/(^|[\s\-(])(\w)/g, (_, start, char) => start + char.toUpperCase()) : 'Unknown suburb');
const formatDistance = (metres) => (metres < 1000 ? `${Math.round(metres)} m` : `${(metres / 1000).toFixed(2)} km`);

function setFinderStatus(message) {
  document.querySelector('#finder-status').textContent = message;
}

// A simple flat pin: blue drop with a white centre, tip at the search point.
const pinIcon = L.divIcon({
  className: 'finder-pin',
  html: '<svg viewBox="0 0 24 32" aria-hidden="true"><path d="M12 1C6 1 1.5 5.5 1.5 11.2 1.5 19 12 31 12 31s10.5-12 10.5-19.8C22.5 5.5 18 1 12 1z" fill="#1754fc"/><circle cx="12" cy="11" r="4" fill="#ffffff"/></svg>',
  iconSize: [28, 38],
  iconAnchor: [14, 37],
});

function placePin(lat, lon, radius) {
  if (!pin) {
    pin = L.marker([lat, lon], { draggable: true, title: 'Search point', icon: pinIcon }).addTo(finderMap);
    pin.on('dragend', () => { const { lat: newLat, lng } = pin.getLatLng(); search({ lat: newLat, lon: lng }); });
  } else {
    pin.setLatLng([lat, lon]);
  }
  if (!radiusRing) radiusRing = L.circle([lat, lon], { radius, color: '#1754fc', weight: 2, fillColor: '#1754fc', fillOpacity: .1, interactive: false }).addTo(finderMap);
  else radiusRing.setLatLng([lat, lon]).setRadius(radius);
}

function resultHtml(item, isAnchor) {
  const status = finderLabels[item.repair_status] ? item.repair_status : 'unfilled';
  const job = item.job_id ? `<a href="/jobs/${item.job_id}">Job #${item.job_id}</a>` : 'Not in a job';
  const change = item.last_change;
  const updated = change
    ? `Updated ${formatDate(change.created_at)} by ${change.source === 'crew' ? 'crew' : 'admin'}${change.crew_distance_m != null ? ` (${Math.round(change.crew_distance_m)} m from pothole)` : ''}`
    : (item.repair_updated_at ? `Updated ${formatDate(item.repair_updated_at)}` : 'No updates yet');
  const after = item.repair_photo_url || change?.photo_url;
  return `<article class="finder-item${isAnchor ? ' anchor' : ''}" data-id="${item.id}">
    <div class="finder-item-head"><strong>Pothole #${item.id}</strong><span class="finder-distance">${isAnchor ? 'Searched pothole' : `${formatDistance(item.distance_m)} from pin`}</span></div>
    <div class="finder-item-status"><span class="status-pill ${status}">${finderLabels[status]}</span><span>${esc(suburbName(item.suburb))}</span></div>
    ${item.repair_reason_label ? `<small>${esc(item.repair_reason_label)}</small>` : ''}
    <small>${job} · ${titleWord(esc(item.severity))} · detected ${formatDate(item.captured_at)}</small>
    <small>${updated}</small>
    <div class="photo-pair">
      <button class="photo-thumb" type="button" data-lightbox="${esc(item.image_url)}" data-caption="Before: detected pothole #${item.id}"><img src="${esc(item.image_url)}" alt="Before"><span>Before</span></button>
      ${after ? `<button class="photo-thumb" type="button" data-lightbox="${esc(after)}" data-caption="After: ${finderLabels[status].toLowerCase()} (pothole #${item.id})"><img src="${esc(after)}" alt="After"><span>After</span></button>` : '<span class="photo-thumb empty"><span>No after photo</span></span>'}
    </div>
  </article>`;
}

function highlightResult(id) {
  document.querySelectorAll('.finder-item').forEach((item) => item.classList.toggle('selected', Number(item.dataset.id) === id));
  resultMarkers.forEach((marker, markerId) => marker.setStyle({ weight: markerId === id ? 4 : 2, color: markerId === id ? '#13243a' : marker.options.fillColor }));
}

async function search(params) {
  const radius = Number(document.querySelector('#finder-radius').value);
  lastSearch = params;
  document.querySelector('#finder-places').hidden = true;
  setFinderStatus('Searching...');
  const query = new URLSearchParams({ ...params, radius });
  try {
    const response = await fetch(`/api/v1/detections/nearby?${query}`);
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || 'Search failed');
    const { lat, lon } = data.center;
    placePin(lat, lon, data.radius_m);
    resultLayer.clearLayers();
    resultMarkers.clear();
    data.results.forEach((item) => {
      const color = finderColors[item.repair_status] || finderColors.unfilled;
      const marker = L.circleMarker([item.latitude, item.longitude], { radius: 8, color, fillColor: color, fillOpacity: .85, weight: 2 })
        .bindTooltip(`Pothole #${item.id} · ${finderLabels[item.repair_status] || 'Unfilled'}`)
        .on('click', () => {
          highlightResult(item.id);
          document.querySelector(`.finder-item[data-id="${item.id}"]`)?.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        })
        .addTo(resultLayer);
      resultMarkers.set(item.id, marker);
    });
    document.querySelector('#finder-results').innerHTML = data.results.map((item) => resultHtml(item, item.id === data.anchor_id)).join('');
    finderMap.fitBounds(radiusRing.getBounds(), { maxZoom: 18 });
    const count = data.results.length;
    setFinderStatus(count ? `${plural(count, 'pothole')} within ${formatDistance(data.radius_m)} of the pin, nearest first.`
      : `No potholes within ${formatDistance(data.radius_m)} of the pin. Try a bigger radius.`);
    if (data.anchor_id) highlightResult(data.anchor_id);
  } catch (error) {
    setFinderStatus(error.message);
  }
}

async function searchAddress(text) {
  setFinderStatus('Looking up address...');
  try {
    const response = await fetch(`/api/v1/geocode?${new URLSearchParams({ q: text })}`);
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || 'Address search failed');
    if (!data.results.length) return setFinderStatus('No matching address in Cape Town. Try adding the suburb, or click the map.');
    if (data.results.length === 1) return search({ lat: data.results[0].lat, lon: data.results[0].lon });
    // Several matches: let the admin pick the right one.
    const places = document.querySelector('#finder-places');
    places.innerHTML = `<p>Which one did you mean?</p>${data.results.map((place, index) => `<button type="button" data-index="${index}">${esc(place.label)}</button>`).join('')}`;
    places.hidden = false;
    places.onclick = (event) => {
      const button = event.target.closest('button[data-index]');
      if (button) { const place = data.results[Number(button.dataset.index)]; search({ lat: place.lat, lon: place.lon }); }
    };
    setFinderStatus('');
  } catch (error) {
    setFinderStatus(error.message);
  }
}

document.querySelector('#finder-form').addEventListener('submit', (event) => {
  event.preventDefault();
  const text = document.querySelector('#finder-query').value.trim();
  if (!text) return;
  if (/^#?\d+$/.test(text)) search({ id: text.replace('#', '') });
  else searchAddress(text);
});
document.querySelector('#finder-radius').addEventListener('change', () => { if (lastSearch) search(lastSearch); });
finderMap.on('click', (event) => search({ lat: event.latlng.lat, lon: event.latlng.lng }));
document.querySelector('#finder-results').addEventListener('click', (event) => {
  if (event.target.closest('a, [data-lightbox]')) return; // links and photos do their own thing
  const item = event.target.closest('.finder-item');
  if (!item) return;
  const id = Number(item.dataset.id);
  highlightResult(id);
  const marker = resultMarkers.get(id);
  if (marker) finderMap.panTo(marker.getLatLng());
});
