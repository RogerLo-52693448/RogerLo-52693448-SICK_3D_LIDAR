# SICK_3D_LIDAR

接收並分析 SICK 3D LiDAR 原始資料，並將資料轉換成車輛形狀。

## 專案狀態

目前已建立 Python AI/ML 專案骨架；尚未指定感測器型號、RAW 資料格式或車輛形狀輸出表示法，因此尚未實作資料匯入與重建流程。

原始與處理後資料請放在 `data/`。資料集及模型產物預設不納入 Git，請勿提交敏感或未授權的資料。

## 快速開始

1. 先閱讀 `AGENTS.md`，並遵循任務前檢查及驗證規則。
2. 建立虛擬環境並安裝專案：

   ```bash
   python -m venv .venv
   . .venv/bin/activate
   python -m pip install -e .
   ```

3. 執行目前的 CLI 骨架：

   ```bash
   sick-3d-lidar
   ```

4. 執行測試：

   ```bash
   PYTHONPATH=src/main/python python -m unittest discover -s src/test
   ```

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
