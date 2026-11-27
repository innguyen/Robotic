# Thư viện cảnh tham chiếu S4.5

Hai cảnh dùng chung bộ vật liệu PBR và các module có kích thước thật. Mỗi cửa kho gồm tường có lỗ mở và bộ cửa cùng pivot. Ở bay giữa, phần cửa được tháo để camera nhìn sâu đến cầu trục. Trong văn phòng, các đoạn ống gió có mặt bích, giá treo, cút nối và đoạn chuyển tiết diện ở mép mái gãy. Tam giác của từng cấu kiện quyết định silhouette, bóng đổ, độ sâu và collision tĩnh.

| Nhóm | Nguồn art | Cách dùng |
| --- | --- | --- |
| Kiến trúc | [Modular Factory Facade](https://polyhaven.com/a/modular_factory_facade), [Urban Apartments Facade](https://polyhaven.com/a/modular_urban_apartments_facade), [Rectangular Airduct](https://polyhaven.com/a/modular_airduct_rectangular_01) | Chọn node glTF riêng, ghép các lỗ mở, cửa, cột, gờ và ống gió theo mét |
| Nhà kho | Cầu trục, xe đẩy, xe nâng tay hai bánh, máy phát | Giữ nguyên các bộ phận và UV, quay và đặt từng thiết bị ở vị trí sử dụng |
| Văn phòng | Laptop, tủ ngăn kéo, sổ giấy và các tờ rời | Giữ góc mở máy, tỷ lệ bàn phím, mặt bàn và cao độ sàn |
| Hư hại và ngoại cảnh | Linoleum Brown, Road Damaged 2, Rubble, Shrub 04, Weed Plant 02, Dead Tree Trunk 02 | Đồng đăng ký albedo và displacement của nền vỡ, mép sàn rách, các lớp đá, gạch, bụi cây và khe mặt đường |
| Trang phục | Margaret Toigo, T-shirt Basic Tucked – Teal/Tan và Wool Pants – Blue | UV trên mesh MakeHuman S4, normal DirectX dẫn xuất từ bump của chính tác giả |
| Hàng cây chắn gió | [Fir Sapling Medium](https://polyhaven.com/a/fir_sapling_medium) | Tám cây ngoài sân tìm kiếm dùng chung một mesh 34.937 tam giác. Bộ bake gom đỉnh trong ô 5 cm theo material slot, giữ opacity của tán lá; xoay và đổi kích thước từng cây |

[Lock nguồn](lock.json) ghi tên tác giả nếu có trong catalogue, trang nguồn, URL từng byte tải về, kích thước, SHA-256 và MD5 nhà cung cấp khi có. Nguồn môi trường của Poly Haven dùng CC0. Bốn ảnh trang phục có nguồn, tác giả và giấy phép riêng trong từng record. Normal áo là sản phẩm dẫn xuất. Các mesh vỏ nhà, cách bố trí và hình học hư hại do dự án tạo được phát hành CC0-1.0. Mesh nhân vật, ghế, bàn và một số thiết bị dùng lại có inventory riêng. [Poly Haven](https://polyhaven.com/license) · [Nhân vật S4](../anatomy/README.md) · [Môi trường S4](../environment/README.md)

## Dựng lại trên Windows

Chạy từ gốc repository với Python 3.11, NumPy 2.2.6 và OpenCV 4.11.0. Máy nghiệm thu có Python đi kèm UE 5.6.1. Các lệnh kiểm tra từng hash, phục hồi nguồn thiếu qua HTTPS, dẫn xuất normal vải, rồi bake mesh theo đúng bố trí. Các đường dẫn output của runner phải mới.

```powershell
$ue = 'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries'
$python = "$ue/ThirdParty/Python3/Win64/python.exe"
$editor = "$ue/Win64/UnrealEditor-Cmd.exe"
$env:PYTHONPATH = "$PWD/.s4-cache/python-packages"
& $python experiments/ue5_s4/prepare_assets.py
& $python experiments/ue5_s4/prepare_environment.py
& $python -m experiments.ue5_s4.bake_reference_sites
& $python -m experiments.ue5_s4.bake_reference_sites --verify
& $python -m sar_benchmark.runner --editor $editor --out experiments/ue5_s4/runs/reference-qa-local --suite sar-smoke --scene-set reference --environments indoor outdoor --cases clean --methods frontier --size 640x480 --scan-rays 720 --action-dt .5 --ticks 4 --render-quality standard --admissibility --calibrate --overview
& $python docs/research/tools/export_s45.py experiments/ue5_s4/runs/reference-qa-local docs/research/evidence/s45/local-qa
& $python -m sar_benchmark.runner --editor $editor --out experiments/ue5_s4/runs/reference-search-local --suite sar-smoke --scene-set reference --environments indoor outdoor --cases clean --methods frontier core verify --size 320x240 --scan-rays 360 --action-dt .5 --ticks 60 --render-quality standard --admissibility --calibrate --overview
& $python docs/research/tools/export_s45.py experiments/ue5_s4/runs/reference-search-local docs/research/evidence/s45/local-search --video e000 e003
```

Trên máy mới, cài hai phiên bản NumPy và OpenCV nêu trên vào interpreter dùng cho runner; plugin PythonScriptPlugin của UE nạp scene bằng editor Python. Runner chụp mã, cấu hình, hai lock môi trường, manifest nhân vật và mesh vào thư mục run trước khi mở editor. Các asset UE được cache theo tên có hash dưới `editor_host/Content/SarAssets`. `--scene-set assets` chọn bố trí S4 đã nghiệm thu; `reference` chọn S4.5. [Runner và hợp đồng sensor](../../../ue5_s4/README.md)

## Thêm hoặc sửa một cấu kiện

1. Khóa glTF, buffer và đúng các atlas PBR của tác giả. Chọn cặp node tường/lỗ mở có cùng pivot. Kiểm số slot của mesh và chiều normal map.
2. Đặt module trong [reference_layouts.py](../../reference_layouts.py) theo FLU mét. Bộ bake bỏ vị trí trưng bày catalogue của từng module, giữ các transform nội bộ của prop nguyên cụm. [Manifest mesh, slot và từng transform](meshes.json)
3. Khai báo footprint bảo thủ của vật cản ở vùng robot có thể đến trong [asset_scenarios.py](../../../../sar_benchmark/asset_scenarios.py). UE dùng collision tam giác cho các cấu kiện tĩnh, kiểm bounds sau import và thực hiện sweep chassis thật.
4. Bake lại, chạy admission, chụp RGB robot và các góc QA, kiểm depth/mask rồi chạy search. Hình overview và đường khảo sát nằm trong vùng dữ liệu riêng của host. [QA thị giác](../../../../docs/research/s45-visual-qa.md)

## Các nguồn đã xem xét

Abandoned Office Park có 315 mesh và example level. Modular Abandoned Factory Environment nêu hỗ trợ UE 5.6 và có bộ tường, ống, cây, vật liệu, scene mẫu. Quixel cũng có bộ Derelict Corridor và cấu kiện công nghiệp hư hại. Muốn dùng bản Fab, đăng nhập Epic, kiểm tra giá và license theo listing, thêm vào My Library, chọn phiên bản UE thích hợp trong Launcher rồi Add to Project hoặc tạo project mẫu để migrate các dependency. Quyền phân phối lại source art phải xét theo license của listing. Bản S4.5 trong repository có thể dựng từ bộ CC0 đi kèm. [Office Park](https://www.fab.com/listings/591840bd-1839-4cf1-8053-b44d73f5ddbe) · [Factory](https://www.fab.com/listings/1cc66fae-b1a1-4dd0-9d5c-0b7b154f4827) · [Quixel](https://www.fab.com/sellers/Quixel%20Megascans?q=dilapidated) · [Quy trình của Epic](https://dev.epicgames.com/documentation/en-us/fab/purchasing-and-downloading-assets-in-fab) · [Fab EULA](https://www.fab.com/eula)

ambientCG cho phép lấy các vật liệu CC0 độc lập. Dùng atlas cùng UV của bộ Poly Haven giữ chi tiết gạch, gờ, khung cửa và các lớp kim loại đồng bộ với mesh tác giả. [Giấy phép ambientCG](https://docs.ambientcg.com/license/)
