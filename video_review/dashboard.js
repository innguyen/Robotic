
    const $ = id => document.getElementById(id);
    $('targetInput').addEventListener('keydown', e => { if (e.key === 'Enter') setTarget(); });
    function setTarget() {
      const t = $('targetInput').value.trim();
      const selected = t ? fetch('/set_target',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:t})}).then(r=>r.json()) : Promise.resolve({status:'ok'});
      selected.then(d=>{if(d.status!=='ok') throw new Error(d.message||'Không thể chọn mục tiêu');if(t){$('targetBadge').textContent=(d.target||t).toUpperCase();$('targetInput').value='';}return fetch('/resume',{method:'POST'});}).then(()=>fetch('/status')).then(refreshStatus).catch(e=>{$('hudAction').textContent=e.message||'Lỗi kết nối';});
    }
    function togglePause(){const paused=$('pauseBtn').classList.contains('btn-resume');fetch(paused?'/resume':'/pause',{method:'POST'}).then(refreshStatus).catch(()=>{$('connectionState').textContent='Lỗi kết nối';});}
    function drive(direction){return fetch('/manual_move',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({direction})}).catch(()=>{});}
    document.querySelectorAll('[data-drive]').forEach(button=>{
      const direction=button.dataset.drive;
      button.addEventListener('pointerdown',e=>{if(direction==='stop'){drive('stop');return;}e.preventDefault();button.setPointerCapture(e.pointerId);drive(direction);});
      ['pointerup','pointercancel','lostpointercapture'].forEach(name=>button.addEventListener(name,()=>{if(direction!=='stop')drive('stop');}));
      button.addEventListener('contextmenu',e=>e.preventDefault());
    });
    function setSensor(id,label,state){$(id).className='sensor'+(state==='Lỗi'?' bad':state==='Chậm'?' warn':'');$(label).textContent=state;}
    function refreshStatus(response){return (response&&response.json?response.json():Promise.resolve(response)).then(d=>{
      $('connectionState').textContent='Đã kết nối';$('hudAction').textContent=d.ai_alive===false?'Luồng AI đã dừng':(d.motion_block&&!d.paused?d.motion_block:d.action_text);
      const batteryPill=$('batteryPill');
      if(d.battery_percent===null||d.battery_percent===undefined){$('batteryValue').textContent='Chưa có dữ liệu';batteryPill.className='pill';batteryPill.title='Chưa nhận được /ros_robot_controller/battery';}
      else{$('batteryValue').textContent=`${d.battery_percent}% · ${Number(d.battery_voltage).toFixed(2)} V`;batteryPill.className='pill'+(d.battery_voltage<10.5?' battery-low':d.battery_voltage<11?' battery-warn':'');batteryPill.title='Phần trăm ước tính tuyến tính theo điện áp 3S LiPo';}
      $('pauseBtn').className='btn '+(d.paused?'btn-resume':'btn-pause');$('pauseBtn').textContent=d.paused?'▶　Tiếp tục':'Ⅱ　Tạm dừng';
      $('targetBadge').textContent=(d.target||'').toUpperCase();$('searchModeBadge').textContent=d.search_mode;$('poseBadge').textContent=`Robot: (${d.pose_x} m, ${d.pose_y} m) · ${d.pose_yaw}°`;
      $('hudTargetPos').textContent=d.target_x===null?'Chưa phát hiện':`(${d.target_x}, ${d.target_y}) m${d.target_seen?' · đang thấy':' · vị trí cuối'}`;
      $('metricVx').innerHTML=`${d.linear_x.toFixed(2)} <small>m/s</small>`;$('metricVy').innerHTML=`${d.linear_y.toFixed(2)} <small>m/s</small>`;$('metricWz').innerHTML=`${d.angular_z.toFixed(1)} <small>°/s</small>`;
      $('aiFpsBadge').textContent=`Cam ${Number(d.camera_fps||0).toFixed(1)} · AI ${d.ai_fps.toFixed(1)} FPS`;
      $('aiFpsBadge').title=`YOLO: ${Number(d.inference_ms||0).toFixed(0)} ms/lượt`;
      const dist=v=>v<800?`${v.toFixed(0)} cm`:'Thoáng';$('hudRadar').textContent=d.oakd_avoidance_enabled?`L ${dist(d.dist_left)} · C ${dist(d.dist_center)} · R ${dist(d.dist_right)}`:'Chỉ quan sát';
      $('hudStrafe').textContent=d.strafe_dir==='LEFT'?'Trượt trái':d.strafe_dir==='RIGHT'?'Trượt phải':'Không';
      const lidarState=d.lidar_age===null?'Lỗi':d.lidar_age>1.5?'Chậm':`Đang chạy · ${Number(d.lidar_scan_span_deg||0).toFixed(0)}°`;
      setSensor('sensorLidar','lidarState',lidarState);setSensor('sensorOdom','odomState',d.odom_age===null?'Lỗi':d.odom_age>1.5?'Chậm':'Đang chạy');setSensor('sensorCamera','cameraState',d.camera_age===null?'Lỗi':d.camera_age>1.5?'Chậm':'Đang chạy');
      $('feedStatusOakd').textContent=d.obs_status||'Đang chờ dữ liệu';
      $('faceState').title=d.voice_error||'Thông báo mục tiêu qua loa USB';
      $('navigationState').textContent=`Quét: ${d.explorer_phase||'—'} · ${Number(d.scan_swept_deg||0).toFixed(0)}° | Bản đồ: ${d.mapping_status||'—'} | Thân xe: ${d.footprint_calibrated?'đã cấu hình':'chưa đo, dùng vùng dự phòng'} | Chiều cao: ${d.depth_clearance||'chưa cấu hình'}`;
      $('navigationState').textContent += ` | Lý do: ${d.navigation_block||d.probe_block||d.plan_reason||'—'} | Bản ${d.build||'—'}`;
      $('navigationState').title=`Nguồn kích thước: ${d.footprint_source||'chưa có'} · ${d.body_length_cm||'?'} × ${d.body_width_cm||'?'} cm`;
      $('faceState').textContent=d.voice_error?('Lỗi loa: '+d.voice_error):(d.announcement||'Chờ tìm thấy mục tiêu');
    }).catch(()=>{$('connectionState').textContent='Mất kết nối';setSensor('sensorLidar','lidarState','Lỗi');setSensor('sensorOdom','odomState','Lỗi');setSensor('sensorCamera','cameraState','Lỗi');});}
    refreshStatus();setInterval(()=>fetch('/status').then(refreshStatus).catch(()=>refreshStatus(null)),1000);
  