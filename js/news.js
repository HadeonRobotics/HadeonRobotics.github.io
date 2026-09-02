async function loadNews() {
    const container = document.getElementById('news-container');
    if (!container) return;

    try {
        const res = await fetch('/data/news.json');
        const posts = await res.json();

        if (!posts.length) {
            showEmpty(container);
            return;
        }

        const grid = document.createElement('div');
        grid.className = 'news-grid';
        grid.innerHTML = posts.map(post => `
            <article class="news-card blueprint">
                <div class="news-date">${post.date}</div>
                <h3>${post.title}</h3>
                <p>${post.summary}</p>
                <a href="/news/${post.slug}.html" class="read-more">Read more →</a>
                <i class="corner tl"></i><i class="corner tr"></i><i class="corner bl"></i><i class="corner br"></i>
            </article>`).join('');
        container.innerHTML = '';
        container.appendChild(grid);
    } catch {
        showEmpty(container);
    }
}

function showEmpty(container) {
    container.innerHTML = `
        <div class="blueprint empty-state">
            <div class="kicker">Connection issue</div>
            <h3>Updates Aren't Loading Right Now</h3>
            <p>Refresh the page, or email us directly and we'll fill you in.</p>
            <a class="btn btn-secondary" href="mailto:info@hadeonrobotics.com?subject=News%20Update" style="padding:10px 20px;font-size:16px;letter-spacing:0.04em;text-transform:uppercase;white-space:nowrap">Email Hadeon</a>
            <i class="corner tl"></i><i class="corner tr"></i><i class="corner bl"></i><i class="corner br"></i>
        </div>`;
}

loadNews();
