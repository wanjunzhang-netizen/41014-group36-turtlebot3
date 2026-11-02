const cfg=window.PROJECT_CONFIG||{};
if(/^https:\/\/github\.com\/[\w.-]+\/[\w.-]+\/?$/.test(cfg.repository||'')){
 const link=document.getElementById('code-link');link.href=cfg.repository;link.textContent='Code on GitHub';document.getElementById('repo-pending').hidden=true;
}
const pending=document.getElementById('video-pending');
if(/^[\w-]{11}$/.test(cfg.youtubeId||'')){
 const frame=document.getElementById('demo-youtube');frame.src='https://www.youtube-nocookie.com/embed/'+cfg.youtubeId;frame.hidden=false;pending.hidden=true;
}else if(cfg.videoFile && (/^static\/videos\/[\w.-]+\.mp4$/i.test(cfg.videoFile)||/^https:\/\//.test(cfg.videoFile))){
 const video=document.getElementById('demo-video');video.src=cfg.videoFile;video.hidden=false;pending.hidden=true;
 video.addEventListener('loadedmetadata',()=>{if(video.duration>300){const warn=document.getElementById('video-warning');warn.textContent='This video exceeds 5 minutes. Shorten it before submission.';warn.hidden=false;}});
 video.addEventListener('error',()=>{pending.textContent='Video could not load. Check the file URL and access permissions before submission.';pending.hidden=false;});
}
fetch('static/results.json').then(r=>{if(!r.ok)throw Error('results unavailable');return r.json();}).then(rows=>{
 document.getElementById('pass-count').textContent=rows.filter(r=>r.passed).length+' / '+rows.length+' passed';
 for(const r of rows){const tr=document.createElement('tr');for(const value of [r.seed,r.lidar_sigma,r.camera_dropout_duration,r.duration_s.toFixed(2),r.position_rmse_m.toFixed(5),r.final_dock_error_m.toFixed(5),r.final_heading_error_deg.toFixed(2),r.passed?'Yes':'No']){const td=document.createElement('td');td.textContent=value;tr.append(td);}document.getElementById('results-body').append(tr);}
}).catch(()=>{document.getElementById('pass-count').textContent='Results could not load';});
