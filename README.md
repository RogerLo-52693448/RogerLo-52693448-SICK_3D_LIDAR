# SICK_3D_LIDAR

接收並分析 SICK 3D LiDAR 原始資料，並將資料轉換成車輛形狀。

## 專案狀態

目前提供 TCP 原始位元組接收與保存，預設連線到 `192.168.0.1:2112`。
此埠依 multiScan270 手冊為 CoLa B 通訊埠，程式只被動接收，不發送設定或掃描訂閱命令。
保存的資料不代表已取得或解碼點雲；設備若不主動傳送資料，程式會逾時並回報失敗。
提供 Compact Type 6/version 1 離線解碼與 XYZ 點雲 PCD 輸出。
尚未實作車輛形狀重建。

原始與處理後資料請放在 `data/`。資料集及模型產物預設不納入 Git，請勿提交敏感或未授權的資料。

## 快速開始

Windows 使用者可在 GitHub 按 **Code → Download ZIP**，解壓縮後，
在包含 `pyproject.toml` 的資料夾開啟 PowerShell：

```powershell
$env:PYTHONPATH = (Resolve-Path .\src\main\python).Path
python -m sick_3d_lidar --host 192.168.0.1 --port 2115 --duration 60
```

您的實機連線測試顯示 TCP 2115 可連線、2112 無法連線，因此以上明確指定
`--port 2115`。程式原始預設仍是 2112；可建立連線不代表已確認量測資料輸出。
需安裝 Python 3.10 或更新版本；以上方式不需要安裝額外套件。

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

安裝套件後也可使用 `sick-3d-lidar convert`。只使用 Python 標準庫，無需額外套件。
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
