/* Pure formatter shared by the dashboard and offline status checks. */
function explainNavigation(d) {
  const n = d.navigation || {}, state = n.state || 'WAIT_NAVIGATION';
  const descriptions = {
    PLANNING: ['Đang tìm đường trong luồng riêng', 'Robot chờ khi chưa có đường. LiDAR và điều khiển tiếp tục cập nhật.'],
    PAUSED: ['Đang tạm dừng', 'Bấm Bắt đầu khi muốn bật điều hướng.'],
    WAIT_NAVIGATION: ['Mất hoặc chưa nhận trạng thái điều hướng', 'Kiểm tra cửa sổ khởi chạy và tiến trình navigation_ros1.py.'],
    WAIT_SCAN: ['Chưa có LiDAR đủ mới và ghép được vị trí robot', 'Xem chi tiết LiDAR bên dưới. Bản đồ đang hiện có thể là dữ liệu cũ.'],
    WAIT_ODOMETRY: ['Chưa có vị trí/hướng robot hợp lệ', 'Kiểm tra nguồn /odom và chi tiết odometry bên dưới.'],
    WAIT_TARGET: ['Chưa có mục tiêu hợp lệ để đi tới', 'Đưa mục tiêu vào camera và chờ xác nhận, khoảng cách và vị trí odom.'],
    WAIT_COLLISION_MONITOR: ['Chưa nhận phản hồi mới từ bộ kiểm tra va chạm', 'Kiểm tra tiến trình collision_monitor_ros1.py trong cửa sổ khởi chạy.'],
    START_BLOCKED: ['Vị trí hiện tại không vượt qua kiểm tra thân xe', 'Kiểm tra vật cản sát thân và vùng chưa quan sát tại vị trí robot.'],
    NO_REACHABLE_APPROACH: ['Chưa tìm được đường tới điểm tiếp cận', 'Kiểm tra các vùng đỏ và vùng chưa quan sát nối giữa robot với mục tiêu.'],
    NO_PATH: ['Không tìm được đường trên bản đồ hiện tại', 'Kiểm tra khe hẹp và vùng chưa quan sát.'],
    GOAL_BLOCKED_OR_UNKNOWN: ['Điểm tiếp cận bị chặn hoặc chưa quan sát', 'Kiểm tra bản đồ quanh mục tiêu.'],
    PATH_BLOCKED: ['Không có đoạn đường tiếp theo thực hiện được', 'Đường có thể đã bị vật cản mới hoặc vùng chưa quan sát chặn.'],
    CONTROL_BLOCKED: ['Chuyển động dự kiến không vượt qua kiểm tra va chạm', 'Có đường đi nhưng bước chuyển động tiếp theo chưa an toàn.'],
    UNOBSERVED_DIRECTION: ['Hướng cần đi nằm trong vùng mù LiDAR', 'Kiểm tra hướng robot và vùng cảm biến quan sát được.'],
    STUCK: ['Robot không tiến triển đủ trong thời gian cho phép', 'Kiểm tra nguyên nhân dừng và vật cản; sau đó Tạm dừng rồi Bắt đầu lại.'],
    ARRIVED: ['Đã tới khoảng dừng trước mục tiêu', 'Robot dừng đúng yêu cầu, không cần tiến sát vật thể.'],
    FOLLOW: ['Đang tạo lệnh bám đường', 'Xem lệnh sau Collision Monitor để biết lệnh được phép gửi; chưa phải xác nhận bánh xe đang chạy.'],
    FOLLOW_PARTIAL: ['Đang tạo lệnh tới điểm trung gian', 'Robot tiếp cận vùng quan sát mới trước khi lập tiếp đường tới mục tiêu.'],
    ROTATE: ['Đang xoay tại chỗ về phía mục tiêu', 'Mục tiêu lệch nhiều so với hướng robot; robot quay trước rồi mới tìm đường.'],
    ALIGN: ['Đã tới khoảng dừng, đang xoay mặt về mục tiêu', 'Robot chỉ xoay tại chỗ, không tiến thêm.'],
    ESCAPE: ['Robot đang nằm trong vùng đệm vật cản, lùi chậm ra xa', 'Robot đi chậm theo hướng ngược với vật gần nhất; Collision Monitor vẫn kiểm tra lệnh này.'],
    RECOVERY_ROTATE: ['Chưa có đường: đang xoay nhẹ để thử hướng khác', 'Robot lệch ±20° so với mục tiêu để vùng mù LiDAR không che lối đi; mục tiêu vẫn trong khung hình.'],
    RECOVERY_WAIT: ['Chưa có đường: đứng yên chờ tìm đường ở hướng mới', 'Nếu cả hai hướng thử đều không có đường, robot sẽ dừng.'],
    SEARCH_SPIN: ['Tìm kiếm: đang xoay quét tại chỗ', 'Robot xoay từng bước 45° và dừng ngắn ở mỗi hướng để YOLO kịp xác nhận mục tiêu.'],
    SEARCH_EXPLORE: ['Tìm kiếm: đang đi tới vùng chưa nhìn thấy', 'Điểm đến (hình thoi vàng) là chỗ có nhiều vùng camera/LiDAR chưa quan sát; tới nơi robot sẽ xoay quét tiếp.'],
    SEARCH_LAST_SEEN: ['Tìm kiếm: mất dấu, đi tới chỗ thấy lần cuối', 'Robot dừng trước vị trí mục tiêu được thấy gần nhất rồi xoay quét lại.'],
    SEARCH_COMPLETE: ['Tìm kiếm xong: không thấy mục tiêu', 'Đã quét hết vùng tới được trong bản đồ hiện tại. Đổi vị trí robot hoặc mục tiêu rồi bắt đầu lại.'],
    FOUND: ['ĐÃ TÌM THẤY MỤC TIÊU', 'Robot đã tiếp cận và dừng; vị trí được đánh dấu đỏ trên bản đồ.'],
    INPUT_CHANGED:['Dữ liệu hoặc chế độ đổi trong lúc tính đường', 'Lệnh cũ đã hủy; chờ chu kỳ cập nhật tiếp theo.'],
    INVALID_GOAL: ['Tọa độ mục tiêu không hợp lệ', 'Chọn lại mục tiêu và kiểm tra dữ liệu định vị.']
  };
  const errors = {
    WAIT_SCAN: 'Chưa có scan được chấp nhận.',
    STALE_SCAN: 'Scan ở Collision Monitor đã quá hạn.',
    STALE_SCAN_STAMP: 'Timestamp LiDAR quá cũ hoặc nằm quá xa trong tương lai.',
    SCAN_NEEDS_MATCHED_ODOMETRY: 'Đã nhận scan nhưng chưa có odometry đủ gần thời điểm scan.',
    SCAN_FRAME_MISMATCH: 'Tên hệ tọa độ LiDAR không khớp cấu hình scan_frame.',
    NONINCREASING_SCAN_STAMP: 'Timestamp LiDAR lặp lại hoặc đi lùi.',
    'Insufficient usable scan rays': 'Quá ít tia LiDAR hợp lệ ngoài vùng mù.',
    'Insufficient usable laser rays': 'Quá ít tia LiDAR hợp lệ ngoài vùng mù.',
    'Invalid scan or pose': 'Thông số scan hoặc vị trí robot không hợp lệ.',
    'Invalid scan metadata': 'Thông số góc/khoảng đo LiDAR không hợp lệ.',
    INVALID_POINTS: 'Điểm LiDAR chứa giá trị không hợp lệ.',
    WAIT_ODOMETRY: 'Chưa nhận odometry hợp lệ.',
    STALE_ODOM_STAMP: 'Timestamp odometry quá cũ hoặc nằm quá xa trong tương lai.',
    NONINCREASING_ODOM_STAMP: 'Timestamp odometry lặp lại hoặc đi lùi.',
    ODOM_FRAME_MISMATCH: 'Tên hệ tọa độ odom/base không khớp cấu hình.',
    INVALID_ODOMETRY: 'Vị trí hoặc hướng robot chứa số không hợp lệ.',
    INVALID_QUATERNION: 'Dữ liệu hướng robot không hợp lệ.',
    NO_CONFIRMED_TARGET: 'Chưa có vật thể vượt qua các bước xác nhận.',
    TARGET_NEEDS_MATCHED_ODOMETRY_OR_RANGE: 'Mục tiêu chưa có khoảng cách hoặc odometry khớp thời điểm ảnh.',
    STALE_TARGET: 'Thông tin mục tiêu đã quá hạn hoặc timestamp không hợp lệ.',
    TARGET_FRAME_MISMATCH: 'Hệ tọa độ mục tiêu không khớp odom.',
    INVALID_TARGET: 'Tọa độ mục tiêu không hợp lệ.',
    INVALID_TARGET_LABEL: 'Tên loại mục tiêu không hợp lệ.',
    STOP_ZONE: 'Có điểm vật cản trong vùng dừng quanh thân xe.',
    PREDICTED_COLLISION: 'Quỹ đạo dự đoán vẫn có va chạm sau khi thử giảm tốc.',
    UNOBSERVED_DIRECTION: 'Lệnh đi/quay hướng vào vùng mù LiDAR.',
    STALE_COMMAND: 'Lệnh vận tốc gửi tới Monitor đã quá hạn.',
    WAIT_COMMAND: 'Monitor chưa có lệnh vận tốc.',
    INVALID_COMMAND: 'Lệnh vận tốc chứa giá trị không hợp lệ.',
    IDLE: 'Lệnh nhận được bằng 0.', PASS: 'Lệnh vượt qua kiểm tra va chạm.',
    LIMIT: 'Đã giới hạn tốc độ tối đa.', SLOWDOWN: 'Đang giảm tốc vì vật cản gần thân.',
    APPROACH: 'Đang giảm tốc thêm để tránh va chạm dự đoán.'
  };
  const seconds = v => Number.isFinite(v) ? v.toFixed(3)+' s' : 'chưa có số liệu';
  const velocity = v => Array.isArray(v) && v.length >= 3 ?
    'tiến '+v[0].toFixed(3)+' m/s; ngang '+v[1].toFixed(3)+' m/s; quay '+v[2].toFixed(3)+' rad/s' : 'chưa có số liệu';
  const detail = code => errors[code] || 'Chưa có diễn giải cho mã này; xem mã kỹ thuật.';
  const monitorBlock = state.startsWith('MONITOR_');
  const description = monitorBlock ? ['Bộ kiểm tra va chạm đang giữ robot dừng', detail(state.slice(8))] :
    (descriptions[state] || ['Trạng thái chưa có diễn giải', 'Xem mã kỹ thuật và dữ liệu chi tiết bên dưới.']);
  const lines = [description[0]+' ['+state+']', description[1]];
  lines.push('Chế độ: '+(n.goal_mode==='point'?'Đi tới điểm chọn trên bản đồ':n.goal_mode==='search'?'Tự động tìm kiếm':'Theo mục tiêu camera'));
  const search = n.search;
  if (search) {
    if (search.found) lines.push('Vị trí mục tiêu (odom): x='+search.found[0].toFixed(2)+' m, y='+search.found[1].toFixed(2)+' m');
    lines.push('Tìm kiếm: đã quét '+search.spots+' điểm, camera đã nhìn ~'+Number(search.covered_m2).toFixed(1)+' m²'+
               (search.nav_reason ? ' · điều hướng: '+search.nav_reason : ''));
  }
  if (n.goal_error) lines.push('Điểm đến chưa được chấp nhận: '+n.goal_error+' — chọn lại điểm trống khi bản đồ mới.');
  if (d.mode === 'preview') lines.unshift('CHẠY THỬ: lệnh không gửi tới động cơ. Dùng run_test_robot_live.bat để chạy thật.');
  for (const [name, key] of [['LiDAR', 'scan_error'], ['Odometry', 'odom_error'], ['Mục tiêu', 'target_error']]) {
    if (n[key]) lines.push(name+': '+detail(n[key])+' ['+n[key]+']');
  }
  const limits = n.limits || {};
  if (state === 'WAIT_SCAN' || state === 'WAIT_ODOMETRY' || monitorBlock) {
    if (state === 'WAIT_SCAN' && !n.scan_error && Number.isFinite(n.scan_age_s))
      lines.push('Scan từng được chấp nhận nhưng hiện không còn đủ mới để điều hướng.');
    if (state === 'WAIT_ODOMETRY' && !n.odom_error && Number.isFinite(n.odom_age_s))
      lines.push('Odometry từng hợp lệ nhưng hiện không còn đủ mới để điều hướng.');
    lines.push('Tuổi scan đã ghép: '+seconds(n.scan_age_s)+' / giới hạn '+seconds(limits.scan_timeout_s));
    lines.push('Tuổi timestamp scan nhận gần nhất: '+seconds(n.scan_header_age_s)+'; tuổi odometry: '+seconds(n.odom_age_s)+' / giới hạn '+seconds(limits.odom_timeout_s));
    lines.push('Lệch odometry gần nhất với scan đang xử lý: '+seconds(n.pending_scan_sync_s)+' / giới hạn '+seconds(limits.pose_sync_tolerance_s));
    lines.push('Xử lý bản đồ: '+(n.map_ms ?? '?')+' ms; bước điều khiển: '+(n.plan_ms ?? '?')+' ms; tìm đường nền: '+(n.async_planner_ms ?? '?')+' ms.');
  }
  if (state === 'WAIT_TARGET') lines.push('Confidence: '+(Number.isFinite(d.confidence)?Math.round(d.confidence*100)+'%':'chưa có')+
    ' / ngưỡng '+(Number.isFinite(d.confidence_threshold)?Math.round(d.confidence_threshold*100)+'%':'?')+'; xác nhận '+(d.hits ?? '?')+'/'+(d.required_hits ?? '?')+' lần.');
  if (n.monitor_state) lines.push('Collision Monitor: '+detail(n.monitor_state)+' ['+n.monitor_state+']');
  lines.push('Lệnh điều hướng: '+velocity(n.velocity));
  lines.push('Lệnh sau Monitor: '+velocity(n.monitor_velocity)+'; tuổi phản hồi '+seconds(n.monitor_age_s));
  return lines.join('\n');
}
if (typeof module !== 'undefined') module.exports = {explainNavigation};
