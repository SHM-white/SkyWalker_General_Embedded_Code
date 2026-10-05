'use strict';

(() => {
  const modules = window.SKYWALKER_MODULES || [];
  const byId = new Map(modules.map(module => [module.id, module]));
  const architecture = window.SKYWALKER_ARCHITECTURE;
  let documents = [];
  let documentMode;
  let documentError = '';
  let documentRequest;
  const categories = window.SKYWALKER_CATEGORIES || [];
  const rootUrl = new URL('../../', window.location.href);
  const content = document.getElementById('content');
  const breadcrumb = document.getElementById('breadcrumb');
  const input = document.getElementById('global-search');
  const statusText = {ready: '已有代码', partial: '接入 / 配置未完成', planned: '目标 / 待实现'};
  let renderVersion = 0;
  let toastTimer;

  const esc = value => String(value ?? '').replace(/[&<>"']/g, char => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[char]));
  const text = value => Array.isArray(value) ? value.join('；') : String(value ?? '');
  const list = value => Array.isArray(value) ? value : [];
  const pill = (status, label) => `<span class="pill ${esc(status || 'neutral')}">${esc(label || statusText[status] || status)}</span>`;
  const routeFile = (path, source = false) => `#${source ? 'source' : 'read'}/${path.split('/').map(encodeURIComponent).join('/')}`;
  const moduleLink = (id, label) => byId.has(id) ? `<a class="relation-chip" href="#module/${esc(id)}">${esc(label || byId.get(id).title)}</a>` : '';
  const sourceLinks = (links, source = true) => `<div class="source-list">${list(links).map(link => `<a href="${routeFile(link.path, source)}">${esc(link.label || link.path)}<small>${esc(link.path)} ↗</small></a>`).join('')}</div>`;
  const plainList = values => `<ul class="plain-list">${list(values).map(value => `<li>${esc(text(value))}</li>`).join('')}</ul>`;
  const empty = message => `<div class="empty-state">${esc(message)}</div>`;
  const heading = (number, title, description, aside = '') => `<div class="page-heading"><div><p class="eyebrow">${esc(number)}</p><h1>${esc(title)}</h1><p class="lead">${esc(description)}</p></div><div class="heading-aside">${aside}</div></div>`;
  const sectionHeading = (title, note = '') => `<div class="section-heading"><h2>${esc(title)}</h2><span>${esc(note)}</span></div>`;
  const downstream = id => modules.filter(module => list(module.depends).includes(id));
  const repositoryFile = path => {
    const parts = path.split('/');
    if (!path || path.startsWith('/') || parts.some(part => part === '..' || part === '.') || path.includes('\\') || /[\u0000-\u001f]/.test(path)) throw new Error('文件路径不合法');
    const url = new URL(parts.map(encodeURIComponent).join('/'), rootUrl);
    if (!url.href.startsWith(rootUrl.href)) throw new Error('文件不在本仓库');
    return url;
  };

  async function loadDocuments() {
    if (documentRequest) return documentRequest;
    documentRequest = (async () => {
      try {
        const response = await fetch(new URL('docs-index.json', window.location.href), {cache: 'no-store'});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const index = await response.json();
        if (!['workspace', 'snapshot'].includes(index.mode) || !Array.isArray(index.documents)) throw new Error('目录格式不正确');
        for (const doc of index.documents) {
          if (!doc || typeof doc.title !== 'string' || typeof doc.path !== 'string' || typeof doc.group !== 'string' || !/\.md$/i.test(doc.path)) throw new Error('文档条目格式不正确');
          repositoryFile(doc.path);
        }
        documents = index.documents;
        documentMode = index.mode;
        documentError = '';
      } catch (error) {
        documentError = `无法加载文档目录：${error.message}。请重试；本地阅读可使用仓库启动脚本。${documentMode ? '当前保留上次成功加载的目录。' : ''}`;
      }
    })();
    try { await documentRequest; } finally { documentRequest = null; }
  }

  const documentWarning = () => documentError ? `<div class="notice warn" role="status">${esc(documentError)} <button class="secondary-button" data-refresh-docs>重试加载</button></div>` : '';

  function nav() {
    document.getElementById('module-nav').innerHTML = categories.map(category => {
      const items = modules.filter(module => module.category === category);
      return items.length ? `<details><summary>${esc(category)} · ${items.length}</summary>${items.map(module => `<a href="#module/${esc(module.id)}" data-module-nav="${esc(module.id)}">${esc(module.title)}</a>`).join('')}</details>` : '';
    }).join('');
  }

  function activateNav(page, id) {
    const section = ({module: 'modules', read: 'docs', source: 'docs', search: 'modules'})[page] || page;
    document.querySelectorAll('[data-nav]').forEach(link => {
      const active = link.dataset.nav === section;
      link.classList.toggle('active', active);
      if (active) link.setAttribute('aria-current', 'page'); else link.removeAttribute('aria-current');
    });
    document.querySelectorAll('[data-module-nav]').forEach(link => {
      const active = page === 'module' && link.dataset.moduleNav === id;
      link.classList.toggle('active', active);
      if (active) { link.setAttribute('aria-current', 'page'); link.closest('details').open = true; }
      else link.removeAttribute('aria-current');
    });
  }

  function wrapSvg(value, width, maxLines = 3) {
    const lines = [];
    let line = '', weight = 0;
    for (const char of String(value || '')) {
      const size = /[^\x00-\xff]/.test(char) ? 1 : .56;
      if (weight + size > width) { lines.push(line); line = ''; weight = 0; }
      line += char; weight += size;
    }
    if (line) lines.push(line);
    if (lines.length > maxLines) { lines.length = maxLines; lines[maxLines - 1] = lines[maxLines - 1].slice(0, -1) + '…'; }
    return lines;
  }

  function vehicleDiagram(mode) {
    const graph = window.SKYWALKER_VEHICLE_MAP?.[mode];
    if (!graph) return '';
    const nodes = new Map(graph.nodes.map(node => [node.id, node]));
    const zoneNames = {gimbal: '云台实时主控', chassis: '底盘实时主控'};
    const zones = ['gimbal', 'chassis'].map(zone => {
      const group = graph.nodes.filter(node => node.zone === zone);
      if (!group.length) return '';
      const x = Math.min(...group.map(node => node.x)) - 13;
      const y = Math.max(1, Math.min(...group.map(node => node.y)) - 30);
      const w = Math.max(...group.map(node => node.x + node.w)) - x + 13;
      const h = Math.max(...group.map(node => node.y + node.h)) - y + 13;
      return `<rect class="vehicle-zone ${zone}" x="${x}" y="${y}" width="${w}" height="${h}" rx="12"/><text class="vehicle-zone-title" x="${x + 13}" y="${y + 18}">${zoneNames[zone]}</text>`;
    }).join('');
    const edges = graph.edges.map(edge => {
      const from = nodes.get(edge.from), to = nodes.get(edge.to);
      if (!from || !to) return '';
      let points = list(edge.points);
      if (points.length < 2) {
        const horizontal = Math.abs(to.x - from.x) > Math.abs(to.y - from.y);
        points = horizontal
          ? [{x: from.x + (to.x > from.x ? from.w : 0), y: from.y + from.h / 2}, {x: to.x + (to.x > from.x ? 0 : to.w), y: to.y + to.h / 2}]
          : [{x: from.x + from.w / 2, y: from.y + (to.y > from.y ? from.h : 0)}, {x: to.x + to.w / 2, y: to.y + (to.y > from.y ? 0 : to.h)}];
      }
      const path = points.map((point, index) => `${index ? 'L' : 'M'}${point.x},${point.y}`).join(' ');
      const center = edge.labelPosition || {x: (points[0].x + points.at(-1).x) / 2, y: (points[0].y + points.at(-1).y) / 2 - 6};
      return `<path class="vehicle-edge ${esc(edge.kind)}" d="${path}" marker-end="url(#vehicle-${esc(edge.kind || 'command')})"><title>${esc(edge.label)}</title></path>${edge.label ? `<text class="vehicle-edge-label" x="${center.x}" y="${center.y}" text-anchor="middle">${esc(edge.label)}</text>` : ''}`;
    }).join('');
    const boxes = graph.nodes.map(node => {
      const titleLines = wrapSvg(node.title, node.w / 12 - 2, 2);
      const detailLines = wrapSvg(node.detail, node.w / 9.5 - 2, 3);
      const body = `<rect x="${node.x}" y="${node.y}" width="${node.w}" height="${node.h}" rx="7"/><circle cx="${node.x + node.w - 12}" cy="${node.y + 13}" r="3"/><text x="${node.x + 12}" y="${node.y + 22}">${titleLines.map((line, index) => `<tspan x="${node.x + 12}" dy="${index ? 16 : 0}">${esc(line)}</tspan>`).join('')}</text><text class="vehicle-detail" x="${node.x + 12}" y="${node.y + 22 + titleLines.length * 16}">${detailLines.map((line, index) => `<tspan x="${node.x + 12}" dy="${index ? 13 : 0}">${esc(line)}</tspan>`).join('')}</text>`;
      return `<g class="vehicle-node ${esc(node.status)}"><title>${esc(node.title)}：${esc(node.detail)}；${statusText[node.status] || ''}</title>${byId.has(node.moduleId) ? `<a href="#module/${esc(node.moduleId)}" aria-label="查看${esc(node.title)}接口">${body}</a>` : body}</g>`;
    }).join('');
    return `<div class="vehicle-map-wrap"><svg class="vehicle-map" viewBox="0 0 1100 ${graph.height || 560}" role="img" aria-labelledby="vehicle-title"><title id="vehicle-title">${mode === 'current' ? '当前实际接入' : '最终上车目标'}：物理分工和数据流。下方提供对应文字链路。</title><defs>${['command', 'feedback', 'permission', 'planned'].map(kind => `<marker id="vehicle-${kind}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path class="arrow-${kind}" d="M0 0 L10 5 L0 10z"/></marker>`).join('')}</defs>${zones}${edges}${boxes}</svg></div>`;
  }

  function overview(params) {
    const mode = params.get('mode') === 'target' ? 'target' : 'current';
    const data = architecture[mode];
    const stats = [{number: '02', title: '实时主控分工', note: '云台决策 / 底盘执行'}, {number: String(modules.length).padStart(2, '0'), title: '模块接口手册', note: '源码、契约与调用顺序'}, {number: String(modules.length), title: '独立接口手册', note: 'Markdown 契约与调用示例'}, {number: documentMode ? String(documents.length) : '—', title: 'Markdown 阅读入口', note: '自动发现正文与样例'}];
    content.innerHTML = heading('01 / System architecture', '从模块到整车，一张图读懂。', `对齐 ${architecture.baseline.branch}@${architecture.baseline.commit.slice(0, 7)} · ${architecture.baseline.date}。先看主控分工，再查公开接口与来源/测量缺口。`, `${pill('partial', '整车装配仍在进行')}<a class="quiet-link" href="#history">查看这次重建的来历 ↗</a>`) +
      documentWarning() + `<div class="intro-stats">${stats.map(stat => `<div class="stat"><div class="stat-number">${stat.number}</div><div class="stat-title">${stat.title}</div><small>${stat.note}</small></div>`).join('')}</div>
      <div class="overview-mode"><div class="tabs" aria-label="架构视图"><a class="${mode === 'current' ? 'active' : ''}" ${mode === 'current' ? 'aria-current="page"' : ''} href="#overview?mode=current">当前实际接入</a><a class="${mode === 'target' ? 'active' : ''}" ${mode === 'target' ? 'aria-current="page"' : ''} href="#overview?mode=target">最终上车蓝图</a></div><div class="legend"><span><i class="ready"></i>已有代码</span><span><i class="partial"></i>接入 / 配置未完成</span><span><i class="planned"></i>目标 / 待实现</span></div></div>
      <div class="panel"><div class="panel-header"><div><h2>${mode === 'current' ? '目前代码真正连通的路线' : '两块实时主控 + 视觉计算机'}</h2><p>点击图中模块进入接口手册 · 箭头表示数据方向</p></div><a class="quiet-link" href="#relations">展开所有模块关系 ↗</a></div>${vehicleDiagram(mode)}<div class="map-footer"><strong>${mode === 'current' ? '当前状态' : '目标约定'}：</strong>${esc(data.summary)}<div class="feedback-lines"><span>→ 命令：带单位的目标与原始时间</span><span>← 反馈：状态 / 参考 / 有效测量</span><span>许可：裁判 + 本地执行条件</span></div></div></div>
      <div class="notice ${mode === 'current' ? 'warn' : ''}">${mode === 'current' ? '<strong>已有代码不等于实车验收。</strong> 两应用共用整车运行时，中央连接与 IMU 安装确认关闭；视觉反馈、功率、热量与拨盘原点仍需真实来源。' : '<strong>此图是工程目标。</strong> 惯性双轴、大 Yaw 与发射框架已有；实物标定和来源闭环待补，搜索/导航仍待实现；绿色仅表示软件存在。'}</div>
      ${sectionHeading('按职责展开调用链', '主图负责分工，下面说明每步的责任')}
      <div class="panel architecture-map"><div class="architecture-lanes" style="--lanes:${data.lanes.length}">${data.lanes.map((lane, index) => `<section class="architecture-lane"><h3 class="lane-heading"><span class="lane-number">${String(index + 1).padStart(2, '0')}</span>${esc(lane.title)}</h3>${lane.steps.map((step, stepIndex) => `${stepIndex ? `<div class="lane-arrow">${mode === 'current' && index === 2 || mode === 'target' && (index === 0 || index === 4) ? '＋' : '↓'}</div>` : ''}<a href="#module/${esc(step.moduleId)}" class="architecture-node ${esc(step.status)}"><strong>${esc(step.title)}</strong><small>${esc(step.note)}</small>${pill(step.status)}</a>`).join('')}</section>`).join('')}</div></div>
      ${mode === 'current' ? `${sectionHeading('距离完整上车，还缺哪些连接？', '从真实应用配置与调用点得出')}<div class="gap-grid">${data.gaps.map(gap => `<article class="gap-card"><h3>${esc(gap.title)}</h3><p>${esc(gap.detail)}</p>${gap.source ? `<a class="quiet-link" href="${routeFile(gap.source, !gap.source.endsWith('.md'))}">查看依据 ↗</a>` : ''}</article>`).join('')}</div>` : `${sectionHeading('完整架构必须固定的契约', '每项都对应一条真实工程边界')}<div class="requirements-grid">${data.requirements.map((requirement, index) => `<article class="requirement"><span class="requirement-index">${String(index + 1).padStart(2, '0')} / INTEGRATION</span><h3>${esc(requirement.title)}</h3><p>${esc(requirement.detail)}</p></article>`).join('')}</div>`}
      ${sectionHeading('建议的阅读顺序')}<div class="reading-path"><a href="#relations"><small>01 / 看关系</small><strong>谁调用谁</strong><p>选择模块，高亮上下游依赖。</p></a><a href="#modules"><small>02 / 查接口</small><strong>如何正确调用</strong><p>参数、返回、时序、线程和示例。</p></a><a href="#workflows"><small>03 / 串主线</small><strong>从输入走到电机</strong><p>逐步阅读遥控、自瞄与恢复链路。</p></a><a href="#docs"><small>04 / 看正文</small><strong>现有 Markdown</strong><p>配置、长篇说明和完整样例。</p></a></div>`;
  }

  function card(module) {
    return `<article class="module-card"><div class="module-category">${esc(module.category)}</div><h3><a href="#module/${esc(module.id)}">${esc(module.title)}</a></h3><p>${esc(module.summary)}</p><small>${esc(module.statusNote)}</small><div class="module-card-footer">${pill(module.status)}<a href="#module/${esc(module.id)}">Markdown 接口与示例 →</a></div></article>`;
  }

  function moduleCatalog(params) {
    const category = params.get('category') || '全部';
    const selection = category === '全部' ? modules : modules.filter(module => module.category === category);
    content.innerHTML = heading('03 / API reference', '每个模块，都能找到具体用法。', '以当前公开头文件和样例为依据。进入模块后，可连续阅读职责、接口契约、调用示例、生命周期和配置边界。', '<a class="quiet-link" href="#read/docs/modules/call-examples.md">完整调用示例手册 ↗</a>') +
      `<div class="toolbar"><div class="filter-buttons" aria-label="模块分类">${['全部', ...categories].map(item => `<button data-category="${esc(item)}" class="${item === category ? 'active' : ''}" aria-pressed="${item === category}">${esc(item)}</button>`).join('')}</div><span class="count-note">${selection.length} 个模块</span></div><div class="module-grid">${selection.map(module => card(module)).join('')}</div>`;
  }

  async function moduleDetail(id, params, version) {
    const module = byId.get(id);
    if (!module) { content.innerHTML = empty('未找到模块。请从“接口与示例”选择现有模块。'); return; }
    await readFile(module.reference, false, params, version, module);
  }

  function relations(params) {
    const selected = byId.get(params.get('selected')) || byId.get('command');
    const columns = window.SKYWALKER_GRAPH_COLUMNS;
    const positions = new Map();
    columns.forEach((column, columnIndex) => column.ids.forEach((id, row) => positions.set(id, {x: 20 + columnIndex * 220, y: 75 + row * 128, w: 184, h: 86})));
    const connected = new Set([...list(selected.depends), ...downstream(selected.id).map(module => module.id)]);
    const edges = modules.flatMap(module => list(module.depends).filter(id => positions.has(id) && positions.has(module.id)).map(id => ({from: id, to: module.id})));
    const paths = edges.map(edge => {
      const a = positions.get(edge.from), b = positions.get(edge.to);
      let path;
      if (a.x === b.x) {
        const sy = a.y + (b.y > a.y ? a.h : 0), ey = b.y + (b.y > a.y ? 0 : b.h);
        const cx = a.x + a.w / 2;
        path = `M${cx},${sy} C${cx - 35},${(sy + ey) / 2} ${cx - 35},${(sy + ey) / 2} ${cx},${ey}`;
      } else {
        const right = b.x > a.x;
        const sx = a.x + (right ? a.w : 0), ex = b.x + (right ? 0 : b.w);
        const sy = a.y + a.h / 2, ey = b.y + b.h / 2;
        const bend = Math.max(32, Math.abs(ex - sx) * .48) * (right ? 1 : -1);
        path = `M${sx},${sy} C${sx + bend},${sy} ${ex - bend},${ey} ${ex},${ey}`;
      }
      const highlighted = edge.from === selected.id || edge.to === selected.id;
      return `<path class="graph-edge ${highlighted ? 'highlighted' : ''}" d="${path}" marker-end="url(#${highlighted ? 'edge-highlight' : 'edge-muted'})"><title>${esc(byId.get(edge.from).title)} → ${esc(byId.get(edge.to).title)}</title></path>`;
    }).sort((a, b) => Number(a.includes('highlighted')) - Number(b.includes('highlighted'))).join('');
    const boxes = modules.map(module => {
      const pos = positions.get(module.id);
      if (!pos) return '';
      const titleLines = wrapSvg(module.title, 13, 2);
      return `<g class="graph-node ${module.id === selected.id ? 'selected' : connected.has(module.id) ? 'connected' : ''}"><a href="#relations?selected=${esc(module.id)}" aria-label="高亮${esc(module.title)}的依赖"><rect x="${pos.x}" y="${pos.y}" width="${pos.w}" height="${pos.h}" rx="8"/><text x="${pos.x + 13}" y="${pos.y + 24}">${titleLines.map((line, index) => `<tspan x="${pos.x + 13}" dy="${index ? 17 : 0}">${esc(line)}</tspan>`).join('')}</text><text class="graph-subtitle" x="${pos.x + 13}" y="${pos.y + 70}">${esc(module.id)}</text></a></g>`;
    }).join('');
    content.innerHTML = heading('02 / Module relationships', '模块各司其职，关系一眼可见。', '箭头从被依赖的模块指向使用它的模块。点击节点高亮直接上下游；这是软件依赖图，整车实时数据流见架构总览和调用链。') +
      `<div class="graph-layout"><div class="panel"><div class="panel-header"><div><h3>模块依赖地图</h3><p>当前选择：${esc(selected.title)}</p></div><a class="quiet-link" href="#relations?selected=application">定位整机应用 ↗</a></div><div class="relation-graph-wrap"><svg class="relation-graph" viewBox="0 0 1120 ${Math.max(610, 185 + (Math.max(...columns.map(column => column.ids.length)) - 1) * 128)}" role="img" aria-labelledby="relation-title"><title id="relation-title">SkyWalker 模块依赖图；绿色箭头为${esc(selected.title)}的直接关系。右侧提供同样的文字链接。</title><defs><marker id="edge-highlight" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path fill="#168674" d="M0 0 L10 5 L0 10z"/></marker><marker id="edge-muted" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto"><path fill="#cad6de" d="M0 0 L10 5 L0 10z"/></marker></defs>${columns.map((column, index) => `<text class="graph-lane-label" x="${20 + index * 220}" y="35">${esc(column.title)}</text>`).join('')}${paths}${boxes}</svg></div><div class="graph-legend"><span>所选模块的直接关系</span><span>其余模块依赖</span></div></div>
      <aside class="panel graph-detail">${pill(selected.status)}<h3>${esc(selected.title)}</h3><p>${esc(selected.responsibility)}</p><div class="relation-label">输入 / 依赖</div><div class="relation-chips">${list(selected.depends).map(id => `<a class="relation-chip" href="#relations?selected=${esc(id)}">${esc(byId.get(id)?.title || id)}</a>`).join('') || '<span class="count-note">直接对接底层设备。</span>'}</div><div class="relation-label">消费者 / 下游</div><div class="relation-chips">${downstream(selected.id).map(module => `<a class="relation-chip" href="#relations?selected=${esc(module.id)}">${esc(module.title)}</a>`).join('') || '<span class="count-note">应用 / 样例直接使用。</span>'}</div><p style="margin-top:20px">${esc(selected.statusNote)}</p><a class="primary-button" href="#module/${esc(selected.id)}">打开接口与调用示例 →</a></aside></div>`;
  }

  function workflows(id, params) {
    const scenarios = architecture.scenarios;
    const scenario = scenarios.find(item => item.id === id) || scenarios[0];
    const requested = Number.parseInt(params.get('step') || '0', 10);
    const index = Math.min(scenario.steps.length - 1, Math.max(0, Number.isFinite(requested) ? requested : 0));
    const step = scenario.steps[index];
    const module = byId.get(step.moduleId);
    content.innerHTML = heading('04 / End-to-end walkthrough', '把接口连起来，才是完整控制链。', '每一步说明谁持有数据、谁负责判定、谁推进硬件。选择场景后可以逐步阅读，并直接进入对应模块。') +
      `<nav class="workflow-picker" aria-label="调用链场景">${scenarios.map(item => `<a class="${item.id === scenario.id ? 'active' : ''}" href="#workflows/${esc(item.id)}" ${item.id === scenario.id ? 'aria-current="page"' : ''}>${esc(item.title)}</a>`).join('')}</nav><div class="notice">${esc(scenario.summary)}</div>
      <div class="workflow-layout"><ol class="workflow-timeline">${scenario.steps.map((item, number) => `<li><a class="${number === index ? 'active' : ''}" href="#workflows/${esc(scenario.id)}?step=${number}" ${number === index ? 'aria-current="step"' : ''}><span class="step-index">${number + 1}</span><span>${esc(item.title)}</span></a></li>`).join('')}</ol><section class="panel workflow-step" aria-live="polite"><p class="eyebrow">Step ${String(index + 1).padStart(2, '0')} / ${scenario.steps.length}</p><h2>${esc(step.title)}</h2><p>${esc(step.detail)}</p>${module ? `<div class="workflow-module">${pill(module.status)}<h3 style="margin-top:12px">${esc(module.title)}</h3><p>${esc(module.summary)}</p><a href="#module/${esc(module.id)}">接口契约与调用示例 →</a></div>` : ''}<div class="workflow-controls"><button class="secondary-button" data-workflow="${esc(scenario.id)}" data-step="${index - 1}" ${index === 0 ? 'disabled' : ''}>← 上一步</button><button class="primary-button" data-workflow="${esc(scenario.id)}" data-step="${index + 1}" ${index === scenario.steps.length - 1 ? 'disabled' : ''}>下一步 →</button><span>${index + 1} / ${scenario.steps.length}</span></div><div class="workflow-source">本场景依据</div>${sourceLinks(scenario.sources.map(source => ({...source})), true).replaceAll('#source/docs/', '#read/docs/')}</section></div>
      ${sectionHeading('从构造到上车：推荐的装配顺序', '这里描述工程顺序，具体命令见正文')}<div class="startup-list">${architecture.startup.map((item, number) => `<article><p class="eyebrow">${String(number + 1).padStart(2, '0')} / SETUP</p><h3>${esc(item.title)}</h3><p>${esc(item.detail)}</p></article>`).join('')}</div>`;
  }

  function docsPage() {
    const groups = [...new Set(documents.map(doc => doc.group))];
    content.innerHTML = heading('05 / Markdown library', '工作区文档，自动汇入目录。', '自动提取 Markdown 标题、路径和目录分组，点击即可在这里阅读。新增、删除或改名文档无需维护索引。', `<span class="pill neutral">${documentMode ? documents.length : '—'} 篇文档 / 样例</span><button class="secondary-button" data-refresh-docs>刷新目录</button>`) +
      documentWarning() + (documentMode ? `<div class="notice">${documentMode === 'workspace' ? '当前目录来自工作区实时扫描，包含尚未提交的 Markdown。进入本页或点击“刷新目录”会重新扫描。' : '当前目录由部署时自动扫描生成，新增文档将在下一次部署后出现；刷新可加载最新部署目录。'}</div>` : '') +
      `<div class="notice"><strong>阅读边界：</strong>模块正文说明现行接口；docs/dev 是设计记录与待实施方案。开发记录中的拟议 API 需要结合当前源码判断。</div>${!documents.length && documentMode ? empty('当前工作区没有可索引的 Markdown 文档。') : ''}<div class="docs-layout">${groups.map(group => `<section class="panel doc-group"><h2>${esc(group)}</h2>${documents.filter(doc => doc.group === group).map(doc => `<a class="doc-entry" href="${routeFile(doc.path)}"><span>${esc(doc.title)}<small>${esc(doc.path)}</small></span><span class="doc-arrow">↗</span></a>`).join('')}</section>`).join('')}</div>`;
  }

  function historyPage() {
    content.innerHTML = heading('06 / Git history', '什么时候变成了覆盖层？', '以下时间来自仓库 Git 记录，均为北京时间。提交记录能说明实际修改与注明的目的，未注明的动机不作猜测。') +
      `<div class="notice warn"><strong>关键启用点：2026-09-30 03:30:47，d860163。</strong> 前一提交添加刷新脚本，该提交把它插入页面。最初为补视觉 / IMU，后续继续叠加命令服务，导致多代描述依靠运行时覆盖。Markdown 目录始终存在。</div><div class="history-timeline">${architecture.history.map(item => `<article class="history-entry"><time>${esc(item.date)}</time><h3><code class="commit">${esc(item.hash)}</code>${esc(item.title)}</h3><p>${esc(item.detail)}</p></article>`).join('')}</div>
      ${sectionHeading('这次重建的维护方式')}<div class="panel panel-body"><ol class="steps-list"><li>一个渲染入口 app.js；接口详情维护在 docs/api 的独立 Markdown，页面读取同一正文。</li><li>主图区分“当前实际接入”和“最终上车目标”，模块代码状态与真实装配状态分开显示。</li><li>Markdown 保留为长篇正文并在浏览器内阅读；本地源码入口直接读取工作区，不再固定到过时 GitHub 提交。</li><li>API 修改时更新 docs/api 和模块主题页；应用接入变化时更新架构数据与主图；每轮按照文档同步清单逐项完成。</li></ol><a class="quiet-link" href="#read/docs/architecture-browser/README.md" style="display:block;margin-top:20px">维护与启动说明 →</a></div>`;
  }

  function searchPage(params) {
    const query = (params.get('q') || '').trim();
    input.value = query;
    const terms = query.toLowerCase().split(/\s+/).filter(Boolean);
    const matches = value => terms.every(term => value.toLowerCase().includes(term));
    const foundModules = terms.length ? modules.filter(module => matches(JSON.stringify(module) + ' ' + (documents.find(doc => doc.path === module.reference)?.searchText || ''))) : [];
    const foundDocs = terms.length ? documents.filter(doc => matches(`${doc.title} ${doc.path} ${doc.group}`)) : [];
    content.innerHTML = heading('Search / 搜索', query ? `“${query}”的搜索结果` : '输入一个模块名或接口名。', '搜索模块说明、Markdown 接口签名、参数、配置、示例和文档标题 / 路径。可用多个关键词缩小范围。') +
      documentWarning() + `<div class="search-results">${foundModules.length ? `<h2>模块 / 接口 · ${foundModules.length}</h2><div class="module-grid">${foundModules.map(module => card(module, query.toLowerCase())).join('')}</div>` : ''}${foundDocs.length ? `<h2>Markdown / 样例 · ${foundDocs.length}</h2><div class="panel doc-group">${foundDocs.map(doc => `<a class="doc-entry" href="${routeFile(doc.path)}"><span>${esc(doc.title)}<small>${esc(doc.path)}</small></span><span>→</span></a>`).join('')}</div>` : ''}${!foundModules.length && !foundDocs.length ? empty(query ? '没有匹配结果。可尝试 CommandManager、snapshot、CAN、参考或 IMU。' : '顶部搜索框支持 / 快捷键。') : ''}</div>`;
  }

  function linkForMarkdown(target, path) {
    try {
      if (/^(https?:|mailto:)/i.test(target)) return target;
      if (/^[a-z][a-z\d+.-]*:/i.test(target)) return '';
      const url = new URL(target, repositoryFile(path));
      if (!url.href.startsWith(rootUrl.href)) return '';
      const relative = decodeURIComponent(url.pathname.slice(rootUrl.pathname.length));
      const fragment = url.hash.slice(1);
      if (/\.md$/i.test(relative)) return routeFile(relative) + (fragment ? `?anchor=${encodeURIComponent(decodeURIComponent(fragment))}` : '');
      if (/\.(cpp|hpp|h|c|conf|overlay|yaml|yml|sh|py|txt|json|dts|dtsi|cmake)$/i.test(relative) || /(^|\/)(Kconfig|CMakeLists\.txt)$/.test(relative)) return routeFile(relative, true);
      return url.href;
    } catch { return ''; }
  }

  function inlineMarkdown(value, path, depth = 0) {
    if (depth > 5) return esc(value);
    const pattern = /(`+)([\s\S]*?)\1|!\[([^\]]*)\]\(([^)]+)\)|\[([^\]]+)\]\(([^)]+)\)|\*\*([^*]+)\*\*|__([^_]+)__|\*([^*]+)\*|~~([^~]+)~~/g;
    let result = '', end = 0, match;
    while ((match = pattern.exec(value))) {
      result += esc(value.slice(end, match.index));
      if (match[1]) result += `<code>${esc(match[2])}</code>`;
      else if (match[3] !== undefined) {
        const url = linkForMarkdown(match[4], path);
        result += url && !url.startsWith('#') ? `<img loading="lazy" src="${esc(url)}" alt="${esc(match[3])}">` : esc(match[3]);
      } else if (match[5] !== undefined) {
        const target = linkForMarkdown(match[6], path);
        const outside = /^(https?:|mailto:)/.test(target) && !target.startsWith(rootUrl.href);
        result += target ? `<a href="${esc(target)}" ${outside ? 'target="_blank" rel="noreferrer"' : ''}>${inlineMarkdown(match[5], path, depth + 1)}</a>` : esc(match[5]);
      } else if (match[7] || match[8]) result += `<strong>${inlineMarkdown(match[7] || match[8], path, depth + 1)}</strong>`;
      else if (match[9]) result += `<em>${inlineMarkdown(match[9], path, depth + 1)}</em>`;
      else result += `<del>${esc(match[10])}</del>`;
      end = pattern.lastIndex;
    }
    return result + esc(value.slice(end));
  }

  function markdown(source, path) {
    const lines = source.replace(/\r/g, '').split('\n');
    const headings = [], ids = new Map();
    const chunks = [];
    const tableCells = line => line.trim().replace(/^\|/, '').replace(/\|$/, '').split(/(?<!\\)\|/).map(cell => cell.trim().replace(/\\\|/g, '|'));
    const isTableDelimiter = line => /^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?\s*$/.test(line);
    const listPattern = /^\s*([-+*]|\d+\.)\s+(.+)$/;
    const isBlock = index => /^\s*$|^\s*(`{3,}|~{3,})|^#{1,6}\s|^\s*>|^\s*([-+*]|\d+\.)\s|^\s*([-*_])(?:\s*\1){2,}\s*$/.test(lines[index] || '') || isTableDelimiter(lines[index + 1] || '');
    for (let index = 0; index < lines.length;) {
      const line = lines[index];
      if (!line.trim()) { index++; continue; }
      const fence = line.match(/^\s*(`{3,}|~{3,})(.*)$/);
      if (fence) {
        const block = [], language = fence[2].trim();
        index++;
        while (index < lines.length && !new RegExp(`^\\s*${fence[1][0]}{${fence[1].length},}\\s*$`).test(lines[index])) block.push(lines[index++]);
        if (index < lines.length) index++;
        chunks.push(`${language === 'mermaid' ? '<div class="unrendered-diagram">此正文中的 Mermaid 原始定义如下；交互架构图见“整车架构”和“模块关系”。</div>' : ''}<div class="code-example"><div class="code-heading"><strong>${esc(language || 'text')}</strong><button data-copy-code>复制代码</button></div><pre><code class="language-${esc(language)}">${esc(block.join('\n'))}</code></pre></div>`);
        continue;
      }
      const title = line.match(/^(#{1,6})\s+(.+?)\s*#*$/);
      if (title) {
        const label = title[2].replace(/[`*_]/g, '');
        const base = label.toLowerCase().replace(/[^\p{L}\p{N}\s-]/gu, '').trim().replace(/\s+/g, '-') || 'section';
        const count = ids.get(base) || 0; ids.set(base, count + 1);
        const id = count ? `${base}-${count}` : base;
        headings.push({id, title: label, level: title[1].length});
        chunks.push(`<h${title[1].length} id="${esc(id)}">${inlineMarkdown(title[2], path)}</h${title[1].length}>`);
        index++; continue;
      }
      if (/^\s*([-*_])(?:\s*\1){2,}\s*$/.test(line)) { chunks.push('<hr>'); index++; continue; }
      if (index + 1 < lines.length && line.includes('|') && isTableDelimiter(lines[index + 1])) {
        const cells = tableCells(line), rows = [];
        index += 2;
        while (index < lines.length && lines[index].trim() && lines[index].includes('|')) rows.push(tableCells(lines[index++]));
        chunks.push(`<div class="table-scroll"><table><thead><tr>${cells.map(cell => `<th>${inlineMarkdown(cell, path)}</th>`).join('')}</tr></thead><tbody>${rows.map(row => `<tr>${cells.map((_, col) => `<td>${inlineMarkdown(row[col] || '', path)}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`);
        continue;
      }
      if (/^\s*>/.test(line)) {
        const quote = [];
        while (index < lines.length && /^\s*>/.test(lines[index])) quote.push(lines[index++].replace(/^\s*>\s?/, ''));
        chunks.push(`<blockquote>${markdown(quote.join('\n'), path).html}</blockquote>`); continue;
      }
      const firstItem = line.match(listPattern);
      if (firstItem) {
        const ordered = /^\d/.test(firstItem[1]), items = [];
        const start = ordered ? Number.parseInt(firstItem[1], 10) : 1;
        while (index < lines.length) {
          const item = lines[index].match(listPattern);
          if (!item || /^\d/.test(item[1]) !== ordered) break;
          let value = item[2]; index++;
          while (index < lines.length && /^\s{2,}\S/.test(lines[index]) && !listPattern.test(lines[index])) value += ' ' + lines[index++].trim();
          const task = value.match(/^\[([ xX])\]\s+(.*)$/);
          items.push(`<li>${task ? `<input type="checkbox" disabled ${task[1] !== ' ' ? 'checked' : ''}> ${inlineMarkdown(task[2], path)}` : inlineMarkdown(value, path)}</li>`);
        }
        const tag = ordered ? 'ol' : 'ul';
        chunks.push(`<${tag}${ordered ? ` start="${start}"` : ''}>${items.join('')}</${tag}>`); continue;
      }
      const paragraph = [line.trim()]; index++;
      while (index < lines.length && !isBlock(index)) paragraph.push(lines[index++].trim());
      chunks.push(`<p>${inlineMarkdown(paragraph.join(' '), path)}</p>`);
    }
    return {html: chunks.join('\n'), headings};
  }

  async function readFile(path, isSource, params, version, module = null) {
    breadcrumb.textContent = `${isSource ? '工作区源码' : 'Markdown'} / ${path.split('/').at(-1)}`;
    content.innerHTML = '<div class="loading" role="status">正在读取本地文件…</div>';
    try {
      const url = repositoryFile(path);
      const response = await fetch(url, {cache: 'no-store'});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const source = await response.text();
      if (version !== renderVersion) return;
      const raw = `<a class="secondary-button" href="${esc(url.href)}" target="_blank" rel="noreferrer">打开原始文件 ↗</a>`;
      const toolbar = `<div class="reader-toolbar"><span>${esc(path)}</span><div><a class="secondary-button" href="#docs">← 文档目录</a>${isSource ? '' : `<a class="secondary-button" href="${routeFile(path, true)}">查看 Markdown 源文</a>`}${raw}</div></div>`;
      if (isSource) {
        content.innerHTML = toolbar + `<pre class="source-view"><code>${source.split('\n').map((line, index) => `<span class="source-line" id="line-${index + 1}"><span class="line-no">${index + 1}</span>${esc(line)}</span>`).join('')}</code></pre>`;
        document.title = `${path.split('/').at(-1)} · SkyWalker 源码`;
      } else {
        const rendered = markdown(source, path);
        const toc = rendered.headings.filter(item => item.level <= 3);
        content.innerHTML = (module ? heading('03 / Module reference', module.title, module.summary, `${pill(module.status)}<a class="quiet-link" href="#relations?selected=${esc(module.id)}">在关系图中定位 ↗</a>`) : '') + toolbar + `${path.startsWith('docs/dev/') ? '<div class="notice warn">这是开发记录或待实施方案。本文中的设计建议不代表当前接口已经实现。</div>' : ''}<div class="reader-layout"><article class="markdown-body">${rendered.html}</article><nav class="section-toc" aria-label="文档目录"><div class="toc-title">文档目录</div>${toc.map(item => `<a style="padding-left:${item.level > 1 ? 13 : 0}px" href="${routeFile(path)}?anchor=${encodeURIComponent(item.id)}">${esc(item.title)}</a>`).join('')}</nav></div>`;
        document.title = `${rendered.headings[0]?.title || path.split('/').at(-1)} · SkyWalker`;
      }
      scrollToAnchor(params.get('anchor') || (isSource && params.get('line') ? `line-${params.get('line')}` : ''));
    } catch (error) {
      if (version !== renderVersion) return;
      content.innerHTML = `<div class="error-panel"><h2>暂时无法读取这个本地文件。</h2><p>${esc(path)} · ${esc(error.message)}</p><p>请用仓库提供的脚本启动静态服务器，目录需覆盖整个仓库，才能同时读取 Markdown 与源码。</p><code>bash docs/architecture-browser/run.sh</code><p style="margin-top:18px">打开脚本打印的地址：<a href="http://127.0.0.1:4173/docs/architecture-browser/">http://127.0.0.1:4173/docs/architecture-browser/</a></p><a href="#docs">返回文档目录 →</a></div>`;
    }
  }

  function scrollToAnchor(anchor) {
    if (anchor) requestAnimationFrame(() => {
      const target = document.getElementById(anchor);
      if (target) target.scrollIntoView({block: 'start'});
    });
  }

  async function render() {
    const version = ++renderVersion;
    const hash = window.location.hash.slice(1) || 'overview';
    const split = hash.indexOf('?');
    const path = split < 0 ? hash : hash.slice(0, split);
    const params = new URLSearchParams(split < 0 ? '' : hash.slice(split + 1));
    let parts;
    try { parts = path.split('/').map(decodeURIComponent); } catch { parts = ['overview']; }
    const page = parts[0], id = parts.slice(1).join('/');
    const labels = {overview: '整车架构', relations: '模块关系', modules: '接口与示例', module: '模块接口', workflows: '端到端调用链', docs: 'Markdown 文档', history: '变更来历', search: '搜索'};
    breadcrumb.textContent = `系统 / ${labels[page] || '文档'}`;
    document.title = `${labels[page] || '架构与接口手册'} · SkyWalker`;
    activateNav(page, id);
    document.getElementById('sidebar').classList.remove('open');
    document.getElementById('menu-toggle').setAttribute('aria-expanded', 'false');
    const previousScroll = window.scrollY;
    if (!params.has('anchor') && page !== 'relations' && !(page === 'workflows' && params.has('step'))) window.scrollTo({top: 0});
    if (!architecture || !modules.length) { content.innerHTML = empty('架构数据未加载，请确认通过 run.sh 打开，并保留同目录全部脚本。'); return; }
    if (['overview', 'docs', 'search'].includes(page) || !labels[page] && !['read', 'source'].includes(page)) {
      if (!documentMode || page === 'docs') content.innerHTML = empty('正在扫描 / 加载 Markdown 文档目录…');
      await loadDocuments();
      if (version !== renderVersion) return;
    }
    switch (page) {
      case 'overview': overview(params); break;
      case 'relations': relations(params); requestAnimationFrame(() => window.scrollTo({top: previousScroll})); break;
      case 'modules': moduleCatalog(params); break;
      case 'module': await moduleDetail(id, params, version); return;
      case 'workflows': workflows(id, params); if (params.has('step')) requestAnimationFrame(() => window.scrollTo({top: previousScroll})); break;
      case 'docs': docsPage(); break;
      case 'history': historyPage(); break;
      case 'search': searchPage(params); break;
      case 'read': await readFile(id, false, params, version); return;
      case 'source': await readFile(id, true, params, version); return;
      default: overview(params); activateNav('overview');
    }
    scrollToAnchor(params.get('anchor'));
  }

  function toast(message) {
    const box = document.getElementById('toast');
    box.textContent = message; box.hidden = false;
    window.clearTimeout(toastTimer);
    toastTimer = window.setTimeout(() => { box.hidden = true; }, 2200);
  }

  async function copy(value) {
    try {
      if (navigator.clipboard?.writeText) await navigator.clipboard.writeText(value);
      else {
        const field = document.createElement('textarea');
        field.value = value; field.className = 'sr-only';
        document.body.append(field); field.select();
        const copied = document.execCommand('copy'); field.remove();
        if (!copied) throw new Error('copy');
      }
      toast('调用示例已复制');
    } catch { toast('复制未完成，可直接选择代码复制'); }
  }

  document.getElementById('search-form').addEventListener('submit', event => {
    event.preventDefault(); window.location.hash = `search?q=${encodeURIComponent(input.value.trim())}`;
  });
  document.getElementById('menu-toggle').addEventListener('click', event => {
    const open = document.getElementById('sidebar').classList.toggle('open');
    event.currentTarget.setAttribute('aria-expanded', String(open));
  });
  document.addEventListener('keydown', event => {
    if (event.key === '/' && !/INPUT|TEXTAREA|SELECT/.test(document.activeElement.tagName) && !document.activeElement.isContentEditable) { event.preventDefault(); input.focus(); }
    if (event.key === 'Escape') { document.getElementById('sidebar').classList.remove('open'); document.getElementById('menu-toggle').setAttribute('aria-expanded', 'false'); input.blur(); }
  });
  content.addEventListener('click', event => {
    if (event.target.closest('[data-refresh-docs]')) { render(); return; }
    const category = event.target.closest('[data-category]');
    if (category) window.location.hash = `modules?category=${encodeURIComponent(category.dataset.category)}`;
    const copyButton = event.target.closest('[data-copy-code]');
    if (copyButton) {
      const code = copyButton.closest('.code-example')?.querySelector('pre code');
      if (code) copy(code.textContent);
    }
    const step = event.target.closest('[data-workflow]');
    if (step && !step.disabled) window.location.hash = `workflows/${step.dataset.workflow}?step=${step.dataset.step}`;
  });
  nav();
  window.addEventListener('hashchange', render);
  render();
})();
