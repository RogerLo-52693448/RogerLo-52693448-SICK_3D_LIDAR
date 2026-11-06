# SICK_3D_LIDAR

接收並分析 SICK 3D LiDAR 原始資料，並將資料轉換成車輛形狀。

## 專案狀態

目前提供 TCP 原始位元組接收與保存，預設連線到 `192.168.0.1:2112`。
此埠依 multiScan270 手冊為 CoLa B 通訊埠，程式只被動接收，不發送設定或掃描訂閱命令。
保存的資料不代表已取得或解碼點雲；設備若不主動傳送資料，程式會逾時並回報失敗。
提供 Compact Type 6/version 1 離線解碼與 XYZ 點雲 PCD 輸出。
另提供固定式 LiDAR 的即時變動物件偵測與每次通過的最佳點雲輸出；
不是車型分類或完整車體重建。

原始與處理後資料請放在 `data/`。資料集及模型產物預設不納入 Git，請勿提交敏感或未授權的資料。

## 快速開始

Windows 使用者可在 GitHub 按 **Code → Download ZIP**，解壓縮後，
在包含 `pyproject.toml` 的資料夾開啟 PowerShell：

```powershell
python -m pip install -e ".[detect]"
$env:PYTHONPATH = (Resolve-Path .\src\main\python).Path
python -m sick_3d_lidar --host 192.168.0.1 --port 2115 --duration 60
```

您的實機連線測試顯示 TCP 2115 可連線、2112 無法連線，因此以上明確指定
`--port 2115`。程式原始預設仍是 2112；可建立連線不代表已確認量測資料輸出。
需安裝 Python 3.10 或更新版本；安裝指令會加入 NumPy 與偵測所需的 SciPy。
若只需接收或離線轉檔，可使用 `python -m pip install -e .`，不必安裝 SciPy。

以下為 Linux/macOS 安裝與執行方式：

1. 先閱讀 `AGENTS.md`，並遵循任務前檢查及驗證規則。
2. 建立虛擬環境並安裝專案：

   ```bash
   python -m venv .venv
   . .venv/bin/activate
   python -m pip install -e .
   ```

3. 在能連到設備的電腦上執行接收（60 秒，10 秒未收到資料則失敗）：

   ```bash
   sick-3d-lidar --host 192.168.0.1 --port 2112 --duration 60
   ```

   未安裝套件時，在專案目錄執行：

   ```bash
   PYTHONPATH=src/main/python python -m sick_3d_lidar --host 192.168.0.1 --port 2112
   ```

   可用 `--connect-timeout`、`--idle-timeout` 調整秒數；用
   `--output output/capture.bin` 指定新檔案。預設檔名含 UTC 時間，
   資料保持 TCP 串流順序原樣保存；TCP 收取區塊並非 telegram 邊界。
   不覆寫既有檔案；中斷或斷線時保留已接收資料，並以非零結束碼回報。
   連線失敗不會建立資料檔；連線成功但沒資料可能留下空檔案。

4. 執行測試：

   ```bash
   PYTHONPATH=src/main/python python -m unittest discover -s src/test
   ```

## RAW 轉 XYZ 點雲 / PCD

把接收的 `.bin` 放在 `output/`，在專案目錄的 PowerShell 執行：

```powershell
$env:PYTHONPATH = (Resolve-Path .\src\main\python).Path
python -m sick_3d_lidar convert .\output\tcp_20261105T090945_229824Z.bin --output-dir .\output\pcd --allow-truncated-tail
```

安裝套件後也可使用 `sick-3d-lidar convert`。NumPy 加速解碼；依上面的安裝指令即可取得。
`--output-dir` 必須是尚未存在的新資料夾，以避免覆寫點雲。
輸出的 binary PCD 可用 CloudCompare 或支援 PCD 的工具開啟。

- 僅支援原廠 SDK 0.5.0 的 Compact Type 6/version 1；每個封包先驗證 CRC32 與完整欄位長度。
- 每個完整 telegram/segment 輸出一個 PCD，檔名含設備序號、frame、segment 與 telegram 序號；不合併不同幀或 interlace 步驟。
- 單位為公尺，角度為弧度；依 SDK 使用
  `x = r*cos(elevation)*cos(azimuth)`、
  `y = r*cos(elevation)*sin(azimuth)`、
  `z = -r*sin(elevation)`，座標尚未轉換至道路或車輛座標系。
- 保留全部回波，依 SDK 的 row/column/echo 順序輸出；略過原始距離為 0 的樣本，
  不代表其餘點都通過有效性或物件分類檢查。PCD 為非組織化點雲（HEIGHT 1）。
- `intensity` 由 12-bit 數值正規化至 0～1；封包未提供強度時填入 NaN。
  環境光、脈衝寬度與 properties 區塊會驗證長度，但不輸出為 PCD 欄位。
- 預設遇到截斷資料即失敗；定時接收可能截斷最後封包，
  `--allow-truncated-tail` 僅允許警告並略過尾端不完整封包，CRC 錯誤與未知格式仍失敗。
- 序號跳號會警告，不補造缺失資料；錯誤前已輸出的 PCD 會保留。
  時間戳、segment/interlace 資訊及略過的零距離點數記錄於 PCD 註解中。

## 固定式 LiDAR 即時物件偵測

在空場、LiDAR 完全固定的條件下開始執行；暖機期間請勿讓目標進入監測區域：

```powershell
python -m pip install -e ".[detect]"
python -m sick_3d_lidar detect --host 192.168.0.1 --port 2115
```

此模式使用 TCP 2115，只接收 Type 6/version 1，不發送設備設定或登入命令。
預設持續執行，按 **Ctrl+C** 停止；可加 `--duration 300` 執行 300 秒。
輸出放在 `output/objects_<UTC時間>/`；也可指定尚不存在的 `--output-dir`。

流程如下：

1. 收齊一個 frame 的所有 segments 才處理；缺失的 frame 只警告略過，
   不當作「沒有物件」。感測器重置或掃描設定改變時停止，需重新建立背景。
2. 每個 interlace 相位收集 10 個空場 frame，合併為固定背景。
   例如 4 相位、20 Hz 約需 2 秒，缺幀時更久；啟動時會顯示進度。
3. 以背景體素中心的最近距離找出新出現的表面，再將變動點依
   26 鄰接體素群聚。只偵測新量測點，不將背景消失產生的空洞當作物件。
4. 以點群中心及簡單速度預測配對追蹤。候選物件至少出現 3 個 frame 才會保存。
   消失至少 1 秒且至少缺席 3 個完整 frame，才視為離開。
5. 每個已確認的 track 只存一個 PCD：選擇該物件在單一 frame 中
   **變動點數最多**的觀測，不混合不同時間或 interlace 相位。
   正常離開時檔名如 `object_000001_sensor_..._frame_....pcd`。

停止、斷線或錯誤時，仍在場的已確認物件會存成 `*_partial.pcd`，
並在 PCD 註解標示 `complete_passage false` 及結束原因，**不當作完整通過**。
沒有足夠觀測次數的候選物件不保存；背景尚未完成就停止會回報失敗。
PCD 包含原始密度的前景 XYZ/intensity（不是僅保存體素中心），可用 CloudCompare 開啟。
偵測模式不另存整段 RAW；需要原始串流時請使用接收模式。

主要調整參數（距離均為公尺）：

| 參數 | 預設 | 用途 |
|---|---:|---|
| `--background-frames` | 10 | 每個 interlace 相位的空場暖機 frame 數 |
| `--voxel-size` | 0.15 | 背景體素邊長，影響速度與空間精度 |
| `--change-distance` | 0.45 | 與背景體素中心的最小差異；不得小於體素邊長的 √3 倍 |
| `--cluster-size` | 0.6 | 26 鄰接群聚的體素邊長；太大可能合併鄰近物件 |
| `--min-points` / `--min-voxels` | 50 / 3 | 點群最少原始點數及群聚體素數，抑制雜訊 |
| `--min-hits` | 3 | 物件 track 至少被配對觀測的 frame 數 |
| `--track-distance` | 2 | 預測中心與當前中心的最大配對距離 |
| `--exit-seconds` / `--min-missed-frames` | 1 / 3 | 必須同時滿足的離開條件，使用感測器時間 |
| `--min-range` / `--max-range` | 0.5 / 80 | 排除過近與過遠的點 |
| `--roi XMIN XMAX YMIN YMAX ZMIN ZMAX` | 全範圍 | 限制監測區，座標仍為感測器座標 |

例如只看感測器前方的指定區域（範圍請依現場座標調整）：

```powershell
python -m sick_3d_lidar detect --roi 2 30 -5 5 -3 3 --min-points 100
```

也可重播先前的 RAW，使用相同偵測流程與感測器時間，不需要連接設備：

```powershell
python -m sick_3d_lidar detect --input .\output\capture.bin --allow-truncated-tail --output-dir .\output\replay_objects
```

重播資料也必須含足夠的空場暖機與物件離場片段；先前短樣本可能不足以暖機或評估偵測效果。
接收與分析使用有界佇列；若處理追不上設備串流，會明確停止並報錯，
不會無限制累積記憶體或默默丟幀。需要降低設備輸出率、限制 ROI 或使用更快的電腦；
限制 ROI 主要降低偵測負擔，完整封包仍需接收與解碼。

**限制**：背景暖機時就存在的物件會被學成背景；設備移動、場景長期變化後需重啟暖機。
樹葉、雨霧與其他變動也可能形成物件；單純強度變化不觸發偵測。
近距物件互相接觸、遮擋或快速交錯時，簡單追蹤可能合併或分裂 track，
無法保證所有現場情況都只存一份。點數最多不等於最完整車體，
輸出也不是車型辨識、網格模型或尺寸量測結果。

## 專案結構

- `src/main/python/sick_3d_lidar/`: Python 核心套件，依 core、models、services、training、inference 等職責分層。
- `src/main/resources/`: 設定、範例資料及靜態資源。
- `src/test/`: 單元、整合測試與測試夾具。
- `data/`: raw、processed、external 及 temp 資料。
- `notebooks/`: 探索、實驗及報告。
- `models/`, `experiments/`: 模型與實驗追蹤。
- `docs/`, `tools/`, `scripts/`, `examples/`: 文件及開發資源。

## 後續需要確認

- SICK LiDAR 的具體型號與資料擷取方式。
- RAW 資料格式、欄位與座標系。
- 車輛形狀輸出的格式、尺度及精度需求。
- 是否需要即時串流處理或離線批次分析。
