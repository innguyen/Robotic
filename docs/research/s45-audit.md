# S4.5 · Audit dựng cảnh SAR

Điểm xuất phát là `5aafef3700ee4405bcb55d235b8af445ef598ab4` trên `feat/benchmark`. Tùy chọn `--scene-set reference` chọn hai bản dựng mới. `assets` chọn các scene S4 v2, còn `legacy` chọn fixture cũ. Các lượt chạy S4.5 là kiểm tra phát triển trên hai cảnh. [Đề xuất S3](benchmark-proposal.md) · [Audit S4](s4-audit.md) · [Nghiệm thu pilot lịch sử](evidence/s4-site/legacy-pilot-completion-01/acceptance.json)

## Cấu kiện và cách tái lập

Thư viện S4.5 khóa 17 nguồn môi trường Poly Haven và các ảnh trang phục CC0. Tổng cộng có 132 file nguồn và dẫn xuất đầu vào, 156.484.921 byte. Từ đó bộ bake tạo 46 mesh, đặt 69 cấu kiện trong văn phòng và 76 ở depot. Các số này đếm riêng lớp dựng mới, không tính lại actor S4 dùng chung như robot, người và thiết bị giữ lại.

Văn phòng sử dụng cửa và khung có chiều sâu từ bộ mặt đứng, trần thả đang vỡ, ống gió còn giá treo, tủ hồ sơ, máy tính mở và giấy trên bàn. Depot có ba bay bốc hàng; từ bay mở nhìn được cầu trục trong xưởng. Xe đẩy, máy phát, đường kỹ thuật, nền đường hư hại và thực vật có mesh riêng. Sàn vỡ dùng trường cao độ và UV từ cùng một nguồn; các mảng bê tông và gạch rời nằm quanh chân vùng gãy. [Cấu trúc level](../../experiments/ue5_s4/reference_layouts.py) · [Danh mục byte nguồn](../../experiments/ue5_s4/assets/environment45/lock.json) · [Transform và mesh dẫn xuất](../../experiments/ue5_s4/assets/environment45/meshes.json)

Pipeline chuyển module glTF sang centimet cho importer OBJ của UE, giữ mapping material slot và kiểm bounds sau chuyển trục FLU. Các cấu kiện tĩnh sử dụng collision theo tam giác; robot dùng chassis swept kinematic. RGB-D, scan, odometry, controls và evaluator đi qua runner S4. Ground truth, mặt nạ, camera QA và phép thử tuyến chỉ nằm phía host. Mỗi run lưu source snapshot, input chính xác, manifest, journal và hai stream lossless tại đường dẫn trong provenance. [Lệnh cài đặt, bake, chạy và xuất ảnh](../../experiments/ue5_s4/assets/environment45/README.md)

| Profile | Công dụng và giới hạn |
| --- | --- |
| D3D11 standard, 640×480 và 720 tia | Đối chiếu S4 và xem tư thế, cửa, bề mặt từ camera robot. Scalability 2, FXAA, bóng động; fixed exposure, motion blur tắt, cap editor 10 callback/s |
| D3D11 standard, 320×240 và 360 tia | Search trên cùng vật liệu và hình học. Giảm số pixel xuống một phần tư và số tia còn một nửa. Mỗi lệnh mang 0,5 s task clock |
| Preview | Runner có preset scalability 0 để chẩn đoán. Các số đo trong báo cáo dùng standard |

Host dùng UE 5.6.1, D3D11 Feature Level 11_1, Intel Iris Xe và i5-1135G7. Renderer dùng raster, HDRI, directional sun và rect fill. Lumen, Nanite và virtual shadow maps tắt trong project này. Màu, bóng, các mảng tường mỏng và ảnh người ở nơi thiếu sáng vẫn cần người đánh giá trên ảnh xuất thực tế. Giới hạn phạm vi robot xấp xỉ 8 × 9 m; cánh nhà và dải cây ngoài vùng chấm điểm cung cấp chiều sâu khi quan sát. [Cấu hình project](../../experiments/ue5_s4/editor_host/Config/DefaultEngine.ini) · [QA thị giác](s45-visual-qa.md)

## Nhật ký các lần thử

Ở bản cuối, preset reference tắt ambient occlusion và reflection trong screen space, giữ bóng đổ của mặt trời, cubemap và vật liệu PBR. Texture đang sử dụng được nạp đầy đủ. Khi chỉ reset robot, host giữ các actor kiến trúc đang ở cùng vị trí để tránh đăng ký lại hình học tĩnh giữa hai lần chụp. Người, odometry, tick và log episode vẫn được reset. [Các CVar thực tế do UE đọc lại](evidence/s45/poses/host-capabilities.json)

- `s45-first-look` chạy UE thành công trên hai cảnh với Frontier, ba tick mỗi cảnh. Lượt này phát hiện tường một mặt, asset chưa biên dịch xong ở tick đầu và thiết bị chồng đống vỡ. [Audit](evidence/s45/first-look/audit.json) · [Hash gốc](evidence/s45/first-look/provenance.json)
- `s45-validation` hoàn thành hai episode, admission hai cảnh và phép thử xuyên tường. Trong 12 tổ hợp tư thế × ánh sáng × cảnh, tất cả đều có mask nạn nhân đủ ngưỡng. Ba tổ hợp lệch RGB giữa các lần reset; các hash scan và depth khớp. Đối chiếu PNG cho thấy ở outdoor standing clean có 128 pixel khác biệt sát đáy ảnh, chênh tối đa 4 mức RGB8. Kết quả kiểm định lượt này là FAIL. [Audit](evidence/s45/iteration-2/audit.json) · [Từng hash](evidence/s45/iteration-2/asset-calibration.json)
- `s45-reference-qa` dừng khi import LOD cây. Phép so bounds phát hiện đỉnh còn trong OBJ dù tất cả tam giác dùng đỉnh đó đã bị loại. Bộ bake đã lọc tam giác suy biến và đánh lại chỉ số các đỉnh thực sự còn dùng. Ngưỡng kiểm bounds giữ nguyên.

Log gốc lưu cả tên mesh và trục bị từ chối. Sau sửa, kiểm tra `bake_reference_sites --verify` khớp từng byte của 46 OBJ và manifest. Bộ `unittest discover -s tests/sar_benchmark` chạy 68 bài và đạt cả 68. [Lỗi import giữ lại](evidence/s45/lod-import/failure.json) · [Hash dựng lại độc lập](evidence/s45/rebake.json)

## Collision và đường đi

Admission trước khi policy chạy tìm thấy 229 ô liên thông trong văn phòng và 232 ở depot. Trong văn phòng, probe được chấp nhận có đường lưới 3,6 m và thấy 1.703 pixel người trong hai lần chụp ở 320 × 240. Ngoài trời, ở đúng góc quay, probe có thể thấy 173 pixel ngay từ vị trí xuất phát. Vì vậy thời gian tìm thấy ở hai cảnh chịu ảnh hưởng của che khuất và vị trí ban đầu. [Admission văn phòng](evidence/s45/active/admission-000.json) · [Admission depot](evidence/s45/active/admission-001.json)

Khảo sát riêng trên host quét hộp 40 × 40 × 30 cm, tâm cao 20 cm, dọc từng đoạn trong thế giới collision UE. Trong văn phòng, chassis qua được hành lang hồ sơ. Đoạn đi thẳng xuyên vách bị chặn ở 2,285 m. Ở depot, cả lối bắc và lối nam từ điểm xuất phát đến cùng một điểm bên kia sân đều thông. Khi quét thẳng vào Jersey barrier, lần va đầu ở 2,267 m. Đây là phép thử tịnh tiến ở yaw cố định; quãng đường do từng policy thực đi được ghi riêng. [Tuyến văn phòng](evidence/s45/active/e000-routes.json) · [Hai tuyến depot](evidence/s45/active/e003-routes.json)

## Tìm kiếm có điều khiển và cảm biến

Trong lượt `s45-active`, mỗi policy có 60 lệnh, tương ứng 30 s nhiệm vụ, với một nạn nhân đứng ở mỗi cảnh. Evaluator ghi nhận kết quả dưới đây từ báo cáo của policy. Cả sáu episode kết thúc bằng FINISH, không có va chạm hoặc báo cáo sai. Các lượt không tìm thấy nạn nhân vẫn tính là bỏ sót. [Kết quả từng episode](evidence/s45/active/results.json)

| Cảnh | Policy | Đúng / số người | Thời điểm báo đúng | Sai số XY | Quãng đường |
| --- | --- | --- | --- | --- | --- |
| Văn phòng | Frontier | 1/1 | 18,0 s | 7,1 cm | 1,66 m |
| Văn phòng | Core | 0/1 | — | — | 0,24 m |
| Văn phòng | Verify | 0/1 | — | — | 1,27 m |
| Depot | Frontier | 1/1 | 1,5 s | 5,7 cm | 1,53 m |
| Depot | Core | 1/1 | 1,0 s | 6,8 cm | 0,12 m |
| Depot | Verify | 0/1 | — | — | 0,10 m |

Trong 12 tổ hợp cảnh, tư thế và ánh sáng ở 320 × 240, ba lần reset độc lập cho RGB, depth và scan giống từng byte trong từng tổ hợp. Mask có ít nhất 3.442 pixel người. Phép thử depth vuông góc với mặt phẳng 0,6/1/2/4 m có sai số tối đa lần lượt 0,313/0,625/1,250/2,500 mm. Các lệnh tiến, ngang, quay và dừng đạt kiểm tra odometry. Host từ chối vị trí cố tình đặt robot trong mesh rồi reset được sang vị trí hợp lệ. [Tư thế và hash](evidence/s45/active/asset-calibration.json) · [Depth và điều khiển](evidence/s45/active/calibration.json) · [Đối chứng va chạm](evidence/s45/active/asset-geometry-calibration.json)

Lượt này ghi đủ 360 packet policy và 430 capture native. Sau khi hoàn tất hiệu chuẩn, editor trả mã 1 lúc đóng; audit toàn lượt là FAIL. Log ghi 27,9 s từ yêu cầu đóng đến khi đóng file log, sát giới hạn chờ 30 s của runner. Runner hiện dành tối đa 120 s cho editor của cảnh reference và ghi sự kiện nếu phải buộc dừng. Dữ liệu và trạng thái gốc được giữ lại. [Audit lượt có mã thoát 1](evidence/s45/active/audit.json) · [Hash và nơi lưu stream](evidence/s45/active/provenance.json)

## Hai lượt xác nhận trên bản dựng cuối
- **640 × 480, 720 tia, hai smoke episode.** Audit `PASS_OFFLINE_AUDIT`, mã thoát UE 0. Đủ 8 packet giao cho policy và 78 capture native, thiếu 0 capture được chấm. Kiểm định cảm biến `PASS_ENGINE_CALIBRATION`. Working set đỉnh của tiến trình UE trong toàn lượt là 1,03 GiB. [Audit](evidence/s45/poses/audit.json) · [Hiệu chuẩn](evidence/s45/poses/asset-calibration.json) · [Kết quả](evidence/s45/poses/results.json)
- **320 × 240, 360 tia, sáu active-search episode.** Audit `PASS_OFFLINE_AUDIT`, mã thoát UE 0. Đủ 360 packet giao cho policy và 430 capture native, thiếu 0 capture được chấm. Kiểm định cảm biến `PASS_ENGINE_CALIBRATION`. Working set đỉnh của tiến trình UE trong toàn lượt là 0,68 GiB. [Audit](evidence/s45/search/audit.json) · [Hiệu chuẩn](evidence/s45/search/asset-calibration.json) · [Kết quả](evidence/s45/search/results.json)

Khi lặp lại cả sáu active-search episode, số báo cáo đúng/sai, thời điểm báo đúng đầu tiên, trạng thái an toàn và lý do dừng của từng episode đều khớp với bảng ở trên.

## Hiệu năng khi thu quan sát

Thời gian đo từ capture đầu đến cuối của từng episode, gồm cả thời gian chờ policy và IPC. Callback/s là tần số callback editor. Obs/s là số khoảng giữa hai lần thu liên tiếp trên mỗi giây thực. Working set và private commit là số lấy mẫu riêng của tiến trình UE trong cửa sổ chấm điểm. [Dữ liệu 640 × 480](evidence/s45/poses/performance.json) · [Dữ liệu 320 × 240](evidence/s45/search/performance.json)

| Cấu hình | Cảnh | Capture | Giây thực | Callback/s | Obs/s | Working set lớn nhất (GiB) | Private commit lớn nhất (GiB) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 640 × 480 | Văn phòng | 4 | 37,3 | 8,45 | 0,08 | 0,27 | 3,47 |
| 640 × 480 | Depot | 4 | 35,9 | 8,11 | 0,08 | 0,31 | 3,51 |
| 320 × 240 | Văn phòng | 180 | 392,4 | 9,15 | 0,45 | 0,54 | 3,45 |
| 320 × 240 | Depot | 180 | 345,1 | 9,20 | 0,51 | 0,49 | 3,46 |

Các lượt đo dùng một máy và một seed, không đo GPU frame time riêng hay VRAM. Ở 640 × 480 mỗi cảnh chỉ có bốn capture được chấm, nên thời gian chờ một lần có ảnh hưởng lớn đến con số obs/s. Các video phát theo đồng hồ nhiệm vụ. Chất lượng hình ảnh còn chờ người duyệt. [Ảnh, video và phiếu duyệt](s45-visual-qa.md)
