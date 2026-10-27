// Shared by every page: formatting helpers, material estimate and the admin sidebar's job lists.
const JOB = JSON.parse(document.body.dataset.job || 'null'); // the job this page is about, if any

// Repair material estimate: pothole area x assumed patch depth x compacted asphalt density, plus wastage.
const REPAIR_DEPTH_M = 0.05;
const ASPHALT_DENSITY_KG_M3 = 2400;
const MATERIAL_WASTAGE = 1.1;

const esc = (value) => String(value).replace(/[&<>'"]/g, (char) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[char]));
const plural = (count, word) => `${count} ${word}${count === 1 ? '' : 's'}`;
const formatDate = (iso) => new Date(iso).toLocaleDateString(undefined, { day: 'numeric', month: 'short' });
const asphaltKg = (areaM2) => areaM2 * REPAIR_DEPTH_M * ASPHALT_DENSITY_KG_M3 * MATERIAL_WASTAGE;
const formatMass = (kg) => kg >= 1000 ? `${(kg / 1000).toFixed(2)} t` : `${Math.round(kg)} kg`;

// Job age: "Open 3d 4h" while a job is open, "Took 1d 2h" once it's complete. Elements carry data-created
// (and data-completed once finished) and are refreshed every minute; data-style="long" spells the units out.
function formatDuration(ms, long = false) {
  const minutes = Math.max(0, Math.floor(ms / 60000));
  const parts = [[Math.floor(minutes / 1440), 'd', 'day'], [Math.floor((minutes % 1440) / 60), 'h', 'hour'], [minutes % 60, 'min', 'minute']]
    .filter(([value]) => value > 0).slice(0, 2);
  if (!parts.length) return long ? 'less than a minute' : '<1 min';
  return parts.map(([value, short, word]) => (long ? plural(value, word) : `${value}${short === 'min' ? ' min' : short}`)).join(' ');
}

function refreshJobAges() {
  document.querySelectorAll('[data-created]').forEach((element) => {
    const created = new Date(element.dataset.created);
    const long = element.dataset.style === 'long';
    if (element.dataset.completed) {
      const took = formatDuration(new Date(element.dataset.completed) - created, long);
      element.textContent = long ? `Completed in ${took}` : `Took ${took}`;
    } else if (element.dataset.done === 'true') {
      element.textContent = 'Completed';
    } else {
      const open = formatDuration(Date.now() - created, long);
      element.textContent = long ? `Open for ${open} (since ${created.toLocaleString(undefined, { day: 'numeric', month: 'short', hour: '2-digit', minute: '2-digit' })})` : `Open ${open}`;
    }
  });
}
const jobAgeAttributes = (job) => `data-created="${esc(job.created_at)}" data-completed="${esc(job.completed_at || '')}" data-done="${job.completed}"`;

const JOB_ICON = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="5" y="4" width="14" height="17" rx="2"/><path d="M9 4V3h6v1M9 10h6M9 14h6M9 18h3"/></svg>';
const DONE_ICON = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="9"/><path d="m8 12.5 2.7 2.7L16 9.8"/></svg>';

function jobLink(job) {
  const title = `${job.name} · ${plural(job.pothole_count, 'pothole')}, ${job.unfilled_count} unfilled · ${formatMass(asphaltKg(job.area_m2))} asphalt (0.25 m repair cuts) · created ${formatDate(job.created_at)}`;
  const count = job.completed ? '' : `<span class="nav-count">${job.unfilled_count} left</span>`;
  return `<a class="nav-item${JOB?.id === job.id ? ' active' : ''}" href="${esc(job.url)}" title="${esc(title)}">${job.completed ? DONE_ICON : JOB_ICON}<span class="nav-label">Job #${job.id}<small>${esc(job.name)}</small><small><span class="job-age" ${jobAgeAttributes(job)}></span></small></span>${count}</a>`;
}

// Jobs move to "Completed" automatically once none of their potholes are left unfilled.
async function loadJobs() {
  const openList = document.querySelector('#job-list');
  const doneList = document.querySelector('#completed-job-list');
  if (!openList) return;
  try {
    const response = await fetch('/api/v1/jobs');
    if (!response.ok) throw new Error('Unable to load jobs');
    const { jobs } = await response.json();
    const open = jobs.filter((job) => !job.completed);
    const done = jobs.filter((job) => job.completed);
    document.querySelector('#job-count').textContent = open.length;
    document.querySelector('#completed-job-count').textContent = done.length;
    openList.innerHTML = open.length ? open.map(jobLink).join('')
      : `<p class="list-empty">${jobs.length ? 'No open jobs.' : 'No jobs yet. Use "Create a job" on the Job Creator Map.'}</p>`;
    doneList.innerHTML = done.length ? done.map(jobLink).join('') : '<p class="list-empty">No completed jobs yet.</p>';
    refreshJobAges();
  } catch (error) {
    openList.innerHTML = `<p class="list-empty">${esc(error.message)}</p>`;
  }
}

document.querySelector('#refresh-page')?.addEventListener('click', () => window.location.reload());
// History page: pick a job to filter straight away, and show times in the viewer's own timezone.
document.querySelectorAll('[data-autosubmit]').forEach((input) => input.addEventListener('change', () => input.form.submit()));
document.querySelectorAll('[data-utc]').forEach((cell) => {
  cell.textContent = new Date(cell.dataset.utc).toLocaleString(undefined, { day: 'numeric', month: 'short', year: 'numeric', hour: '2-digit', minute: '2-digit' });
});
// Photo lightbox: any element with data-lightbox="<image url>" (and optional data-caption) opens it in-page.
const lightbox = document.querySelector('#lightbox');
function closeLightbox() { if (lightbox) lightbox.hidden = true; }
document.addEventListener('click', (event) => {
  const trigger = event.target.closest('[data-lightbox]');
  if (trigger && lightbox) {
    document.querySelector('#lightbox-image').src = trigger.dataset.lightbox;
    document.querySelector('#lightbox-caption').textContent = trigger.dataset.caption || '';
    lightbox.hidden = false;
  } else if (event.target === lightbox || event.target.closest('#lightbox-close')) {
    closeLightbox();
  }
});
document.addEventListener('keydown', (event) => { if (event.key === 'Escape') closeLightbox(); });

loadJobs();
refreshJobAges();
setInterval(refreshJobAges, 60000);
