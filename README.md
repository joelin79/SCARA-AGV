## SCARA‑AGV 物件偵測與搬運系統
使用時務必將遠離工作範圍，避免手臂無預期的砍人（或電視）！
整合 Intel RealSense 深度相機、YOLO 目標偵測與 SCARA 手臂控制，能自動掃描整個工作區、偵測盤面上的物件，轉換成手臂座標，並支援互動式搬運。

### 功能亮點
- 自動網格掃描：規劃可達的相機位置，完整覆蓋工作區
- YOLO 偵測：載入自訓練的 `yolo/my_model/my_model.pt` 權重
- 深度與座標轉換：像素點 + 深度 → 手臂基座座標 (mm)
- ArUco 盤面偵測：以邊框標記推導盤面範圍，只保留盤內物件
- 去重與品質處理：重疊視角去重、邊界檢測懲罰、置信度排序
- 3D 視覺化：輸出偵測點雲與盤面模型圖
- 互動式搬運：依盤面編號把物件搬移到指定盤

---

## 安裝需求

### 硬體
- SCARA 手臂（含延伸臂/吸頭/相機支架）
- Intel RealSense D435/D435i 深度相機

### 軟體與環境
- Python 3.8 以上（建議 3.9+）
- 套件：
  - `ultralytics`（YOLO）
  - `pyrealsense2`
  - `opencv-python`
  - `numpy`
  - `matplotlib`

安裝範例：
```bash
python -m venv .venv
source .venv/bin/activate  # Windows 用 .venv\\Scripts\\activate
pip install ultralytics pyrealsense2 opencv-python numpy matplotlib
```

---

## 專案結構（重點檔案）
```
SCARA-AGV/
├── main.py                          # 一鍵流程：校正 → 偵測 → 搬運互動
├── object_mover.py                  # 互動式搬運介面（依盤面搬移物件）
├── Detection_Models/
│   ├── ObjectDetection.py           # 主要偵測管線（相機/YOLO/盤面/輸出/視覺化）
│   └── yolo_detection.py            # 單圖/資料夾 YOLO 偵測工具
├── RealSense/
│   ├── realsense_depth.py           # 相機介面
│   ├── camera_calibration.py        # 相機/手眼標定程式
│   └── camera_calibration/*.json    # 標定檔（程式會讀取 factory.json）
├── Arm_Control/
│   └── SCARA.py                     # 手臂控制（串口/運動/安全界限）
├── detection_output/                # 偵測輸出（JSON、影像、3D 圖）
└── CALIBRATION_GUIDE.md             # 詳細相機/手眼標定教學
```

---

## 快速開始（建議流程）（注意安全）

1) 放好 YOLO 權重檔
- 將訓練好的模型放至 `yolo/my_model/my_model.pt`
- 好用的 train 工具：[Google Colab](https://colab.research.google.com/github/EdjeElectronics/Train-and-Deploy-YOLO-Models/blob/main/Train_YOLO_Models.ipynb#scrollTo=43ypwonynLVu)

2) 相機/手眼標定（建議）（用現在預設的也行）
```bash
python RealSense/camera_calibration.py
```
- 產生或更新 `RealSense/camera_calibration/camera_calibration_factory.json`
- 也可參考 `CALIBRATION_GUIDE.md`

3) 一鍵執行
```bash
python main.py
```
- 會自動嘗試：載入手臂控制 → 啟動偵測管線 → 匯出結果 → 啟動搬運互動

完成後輸出在 `detection_output/`：
- `detected_objects.json`：完整偵測與盤面結果
- `captured_images/`：各掃描點的原始與標註影像
- `3d_detection_results.png`、`3d_plate_results.png`：3D 視覺化

---

## 進階使用

### 只跑偵測管線
```bash
python Detection_Models/ObjectDetection.py
```

程式中可自訂：
```python
from Detection_Models.ObjectDetection import ObjectDetectionSystem

detector = ObjectDetectionSystem(
    model_path="yolo/my_model/my_model.pt",
    use_calibration=True,
    save_images=True,
)

detector.initialize()
detector.plan_scanning_positions(
    scan_height=280.0,
    grid_spacing=80.0,
    camera_direction=-90.0,
    y_max=0,
)
detector.scan_workspace(confidence_threshold=0.7)
detector.save_results("detected_objects.json")
detector.visualize_3d()
```

### YOLO 單獨推論（對圖片/資料夾）
```bash
python Detection_Models/yolo_detection.py \
  --model yolo/my_model/my_model.pt \
  --source "path/to/images/*.jpg" \
  --resolution 1280x720
```

### 互動式搬運
偵測完成後（已有 `detected_objects.json`）可執行：
```bash
python object_mover.py
```
- 顯示盤面摘要、物件列表，支援將單一或全部物件搬到指定盤
- 若無法連線手臂，會自動切換為模擬模式

---

## 標定與座標轉換（重點）

- 內部優先讀取 `RealSense/camera_calibration/camera_calibration_factory.json`。
- 若有手眼標定（`handeye_R_ee2cam`, `handeye_t_ee2cam`）會走完整外參轉換；否則退回簡化模型。
- 未標定時仍可運作，但座標精度會下降；建議依 `CALIBRATION_GUIDE.md` 完成內/外參與手眼。

常見需求：
- 內參：`fx/fy/cx/cy`、畸變係數，用於像素反投影與去畸變
- 外參/手眼：相機座標 → 手臂基座座標的旋轉/平移

---

## 輸出與 JSON 結構（節錄）

`detection_output/detected_objects.json` 範例鍵值：
```json
{
  "scan_metadata": {
    "timestamp": "2024-01-01 12:00:00",
    "total_objects": 5,
    "total_plates": 2,
    "scan_positions": 25,
    "model_path": "yolo/my_model/my_model.pt"
  },
  "detected_objects": [
    {
      "object_id": 0,
      "class_name": "block",
      "confidence": 0.85,
      "bbox": [100, 150, 200, 250],
      "center_pixel": [150, 200],
      "depth_mm": 580.5,
      "arm_coordinates": {"x": 250.3, "y": -45.7, "z": 25.8},
      "camera_position": {"x": 250.0, "y": -50.0, "z": 150.0}
    }
  ],
  "detected_plates": [
    {
      "plate_id": 1,
      "center_arm_coordinates": {"x": 100.0, "y": 200.0, "z": 20.0},
      "dimensions": {"width_mm": 200.0, "length_mm": 300.0},
      "height_mm": 18.0,
      "edge_marker_ids": [1, 0, 3, 2],
      "corners_arm_coords": [[...], [...], [...], [...]]
    }
  ]
}
```

---

## 疑難排解（FAQ）

- 相機無法啟動：請安裝 `pyrealsense2`，確認 USB 連線；以 RealSense Viewer 測試
- 找不到模型：確認 `yolo/my_model/my_model.pt` 是否存在
- 無偵測結果：降低 `confidence_threshold`（例如 0.4），改善照明
- 盤面為空：確保 ArUco 邊框標記清楚可見；未偵測到盤面時系統會丟棄所有物件
- macOS USB 攝影機索引：若使用 `yolo_detection.py` 的 `--source usbX`，在 macOS 會被轉成 `1`；建議以影像檔/資料夾測試
- 手臂通訊：確認 `Arm_Control/SCARA.py` 的串口設定與連線狀態

---

## 安全注意事項
- 作業前先確認手臂活動範圍與安全界限
- 自動掃描時務必有人在旁監看並有急停裝置
- 初期請以較高掃描高度、較慢速度驗證流程

---

## 參考文件
- `CALIBRATION_GUIDE.md`：完整內/外參與手眼標定流程與評估
- `OBJECT_DETECTION_README.md`、`README_ObjectDetection.md`：物件偵測與系統架構補充

