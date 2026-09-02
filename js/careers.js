async function loadJobs() {
    const container = document.getElementById('jobs-container');
    if (!container) return;

    try {
        const res = await fetch('/data/jobs.json');
        const jobs = await res.json();

        if (!jobs.length) {
            showEmpty(container);
            return;
        }

        const list = document.createElement('div');
        list.className = 'jobs-list';
        list.innerHTML = jobs.map(job => `
            <div class="job-card blueprint">
                <div class="job-info">
                    <h3>${job.title}</h3>
                    <div class="job-meta">
                        <span class="job-tag">${job.type}</span>
                        <span class="job-tag">${job.location}</span>
                    </div>
                    ${job.description ? `<p class="job-description">${job.description}</p>` : ''}
                </div>
                <a href="${job.applyLink}" class="btn btn-primary">Apply</a>
                <i class="corner tl"></i><i class="corner tr"></i><i class="corner bl"></i><i class="corner br"></i>
            </div>`).join('');
        container.innerHTML = '';
        container.appendChild(list);
    } catch {
        showEmpty(container);
    }
}

function showEmpty(container) {
    container.innerHTML = `
        <div class="blueprint empty-state" style="text-align:left">
            <div class="kicker">Open roles · 0</div>
            <h3>No Posted Positions Right Now</h3>
            <p style="margin:0 0 20px">We hire opportunistically. Send what you've built — solver work, controls, perception, or industrial deployment experience.</p>
            <a class="btn btn-primary" href="mailto:info@hadeonrobotics.com?subject=Open%20Application" style="padding:11px 20px;font-size:16px;letter-spacing:0.04em;text-transform:uppercase;white-space:nowrap">Send an open application</a>
            <i class="corner tl"></i><i class="corner tr"></i><i class="corner bl"></i><i class="corner br"></i>
        </div>`;
}

loadJobs();
