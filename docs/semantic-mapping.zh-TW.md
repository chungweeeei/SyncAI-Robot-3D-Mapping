# 語意建圖：設計筆記

> 本文為 [`semantic-mapping.md`](semantic-mapping.md) 的繁體中文翻譯。英文版為正本，修改時請先改英文版，再同步本檔；若兩者不一致，以英文版為準。

2026-10-15 討論如何在 `pgo_node` 的地圖上加入影像語意的筆記。這裡的內容尚未實作；它記錄的是做了哪些決定、為什麼，以及還缺什麼。目前的管線只用 LiDAR：從頭到尾都是 `pcl::PointXYZI`，沒有訂閱相機，repo 內和 `SyncAI-Robot-Workspace` 的 `description/G23.urdf` 中都沒有任何內參或外參。

## 目標

照常產出 `map.pcd` / `octomap.bt`，並在旁邊另外產出一張地圖，其中每個點（或體素）都帶有來自相機的語意類別：地板 / 樓梯 / 草地 / 牆 / 門 / 人等等，用於可通行性判斷以及「去 X 那裡」這類任務。先做封閉集合的標籤；CLIP 特徵（開放詞彙）體素留作日後的選項。

## 決定：2D 分割 + 投影，附加在關鍵幀上

2D 影像模型提供語意，LiDAR 提供幾何，Point-LIO 提供位姿。不需要 3D 網路（CLIP2Scene 式的蒸餾只有在需要相機視野外的語意時才划算，而且必須先針對每個平台做預訓練）。找到的所有近期系統（見參考資料）都是同一套做法。

```
/<robot_id>/pointlio/body_cloud_dense  (PointXYZI, body frame)
/<robot_id>/pointlio/lio_odom
<camera>/image_raw/compressed  +  K, D, T_cam_lidar
            |
   new node: semantic_labeler
   +- interpolate lio_odom to the image stamp
   +- project points into the image, z-buffer per pixel against occlusion
   +- label each point (class id, or a CLIP feature)
            |
   body_cloud_semantic (PointXYZRGBL or an extra field)
            |
   pgo_node unchanged in behaviour; patches/<i>.pcd now carry the label
            |
   after save_maps: fuse per voxel with the corrected poses.txt
   -> map_semantic.pcd  +  octomap.bt (occupancy only)  +  voxel -> class histogram
```

**為什麼把語意附加在關鍵幀上，而不是世界座標系的體素上。** `pgo_node` 在迴環閉合時會修正關鍵幀位姿。若在建圖過程中於 `map` 座標系累積語意體素網格，修正後它就會和點雲錯位。改為標註 body 座標系的 patch，語意就會跟著關鍵幀一起移動；而 `patches/ + poses.txt` 本來就是這類存檔後處理的交接介面（`clean_map` 也是這樣運作）。下方 2026 年那篇 LIO 加 RGB 建網格的論文正是因為這個原因完全不做迴環閉合；我們不必如此。

**融合規則。** 每個體素保留一個類別直方圖（或對類別做離散貝氏濾波），取 argmax。**不要**把標籤編碼成 `ColorOcTree` 中的偽色再取平均：多幀平均會把顏色混成不存在的類別（2025 年的 Semantic OctoMap 比較論文報告的正是這個問題）。

## 本 repo 中受影響的地方

- 點的型別全程都是 `pcl::PointXYZI`（`include/syncai_mapping/pgos/commons.h`）。要嘛改用帶標籤的型別，要嘛另外帶一個與點索引對齊的標籤陣列。
- `clean_map` 可以直接使用標籤：被分類為 `person` 的點直接丟棄，不必依賴光線投射的 miss 比例。
- `octomap.bt` 維持純佔據資訊。語意放在獨立的 `voxel -> histogram` 表（或第二個帶 `label` 欄位的 PCD），不放進樹裡。
- 跨 repo 的契約（topic、service、`/dev/shm/syncai_pgo/<robot_id>`）維持不變；標註器位於 `syncai_pointlio` 和 `pgo_node` 之間，發布一個新 topic。

## 2D 模型選擇

| 模型 | 類別 | 輸出 | 擅長 | 備註 |
|---|---|---|---|---|
| Mask2Former (ADE20K) | 固定（150 類） | 逐像素標籤 | things 與 stuff | 快、穩定，有 HF/Detectron2 權重；ADE20K 已包含 floor、wall、stairs、grass、door、person |
| Grounded-SAM / SAM 2 | 任意文字 | 實例遮罩 | 物體（things） | 重；對無固定形狀的 stuff（地板、草地）較弱；最適合物體層級的地圖 |
| OpenSeg / SAN / CAT-Seg / MaskCLIP | 任意文字 | 逐像素 CLIP 特徵 | 建圖後的開放詞彙查詢 | 體素很吃記憶體；OpenSeg 只有 TF 版本 |

先用 Mask2Former ADE20K 處理可通行性相關類別；之後再加 Grounded-SAM 做物體實例；只有當事後文字查詢成為需求時才做特徵地圖。2026-10-17 已讓七個後端跑過同一套投影與融合，在我們的資料上真正拉開差距的是什麼，見下方「2D 後端比較」。

## 相機：現有的與缺少的

相機是 TechNexion **VCS-AR0234-C**，1280x720，在 Workspace 中有兩種驅動方式：

| 路徑 | 發布 | `camera_info` |
|---|---|---|
| `vizionsdk_ros2`（`bringup.launch.py use_camera:=true`） | `image_raw/compressed`（MJPG）、IMU | 有，但只在 `VxGetIntrinsics` 成功時（`vizionsdk_camera_node.cpp`、`LoadCameraInfo`）；否則只有一則警告，什麼也不發 |
| `syncai_camera_feeder`（GStreamer / WebRTC） | 只有 `image_raw/compressed` | 無 |

做任何投影之前還需要：

1. **內參** `K`、`D`、影像尺寸。用 `ros2 bag info` 檢查 bag 中是否有 `camera_info` topic。若有，要驗證它：SDK 的數值是模組的出廠校正，可能是在不同的原生解析度下（第二台機器回報的是 1280x800），所以裁切到 720 會讓 `cy` 偏移。對一幀做去畸變並檢查直線，再疊上一次投影後的掃描。若沒有或不正確，就用棋盤格校正：
   `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=<image topic>`。
2. **外參** `T_cam_lidar`。不在 URDF 中，也不在任何參數檔中。用 [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) 校正（輸入 bag，支援 Mid-360，需先有內參）。
3. **位姿內插**到影像時間戳。相機和 LiDAR 的時間戳不同，行走中的四足機器人在兩者之間的移動量足以造成投影錯誤。
4. **一定要檢查疊圖**（把掃描投影到影像上），再在上面建構任何東西。

影像是 JPEG；離線處理用 `cv2.imdecode` 就夠了。

## 已知陷阱

- 遮擋 / 視差：相機和 LiDAR 位置不同，所以相機看不到的點會拿到前景的標籤，除非用逐像素深度測試只保留最近的點。
- 遮罩邊緣：投影誤差加上不精確的遮罩邊界會讓邊界上的點被標錯。可侵蝕遮罩，或降低靠近標籤邊界之點的權重。
- 動態物體會留下殘影；讓 `person` 體素衰減，或乾脆不寫入。
- 只有相機視錐內的點會拿到標籤；其餘的維持未標註，直到有 360 相機或 3D 網路（CLIP2Scene / LOSC）補上。

## 計畫

1. **先離線，不改 ROS。** Bag -> Point-LIO + PGO -> 帶 `save_patches` 的 `save_maps` -> 每個關鍵幀挑最近的影像 -> Mask2Former -> 把標籤投影到 patch 上 -> 用 `poses.txt` 組合 -> 逐體素多數決 -> 輸出上色的語意 `.ply` 供 CloudCompare / RViz 檢視。純 Python；驗證整個構想最快。
2. 把標註器升級為 Point-LIO 與 `pgo_node` 之間的 ROS 2 node；更改點型別。
3. 之後才考慮用 Grounded-SAM 做物體實例，或用 CLIP2Scene 做視野外語意。

## 第一次離線結果（2026-10-16，`dp1f_with_image`）

計畫第 1 步，在 `record/dp1f_with_image` 上端到端執行（784 秒，Mid360 + IMU，相機 30 Hz，`camera_info` 來自棋盤格校正，`tf_static` 帶的是 `G23.urdf` 中暫定的手量安裝位置）。腳本與確切指令：`tools/semantic_offline/README.md`。輸出：`map/dp1f_with_image/semantic/`（`map_semantic.{pcd,ply}`、以清理後 `map.pcd` 遮罩過的 `_clean` 版本、`*_stats.json`、俯視圖、`qa/` 疊圖）。

| 步驟 | 做了什麼 | 結果 |
|---|---|---|
| 重播 | 在機器人映像中跑 pointlio + `pgo_node`（`start_on_launch`），帶 patches 的 `save_maps`，`clean_map` | 1071 個關鍵幀、10 個迴環、8.2 M 點；clean 在 51 秒內 `ok` |
| 關鍵幀時間戳 | `poses.txt` 沒有時間戳。重播時記錄 `lio_odom` 和 pgo 的 `map → pointlio_odom` TF（每個處理過的配對一筆），再對處理過的配對重跑 `isKeyPose` | 1071 = 1071，相鄰相對運動最大不一致 1 mm，637 個 `loop_markers` 時間戳全部找到 |
| 外參檢查 | `overlay_test.py`：在 4 個時刻把原始掃描投影到最近的影像上 | 在 2–3 m 處地板 / 牆的邊界吻合到幾個像素內；手量的安裝位置可用於 stuff 類別，但還不足以處理細小物體 |
| 2D 模型 | Mask2Former Swin-L ADE20K（`transformers`），短邊 512 px | 在 Mac 的 MPS 上每張 0.33 秒（Swin-T：0.21 秒）；測試幀上 floor / wall / ceiling / door / person 都很乾淨 |
| 投影 | 每個關鍵幀取最近的影像（平均 \|dt\| 7 ms，最大 35 ms），在影像時間戳透過 `lio_odom` 內插 body 位姿，8 px z-buffer 防遮擋，標籤邊界上權重 × 0.3 | 每個 patch 有 16 % 的點落在視錐內；1071 個關鍵幀耗時 326 秒 |
| 融合 | 0.1 m 體素，加權類別直方圖，argmax | 1.34 M 個體素，31 % 有標籤（以清理後 `map.pcd` 遮罩後剩下的 0.90 M 體素中有 40 %），平均投票純度 0.95 |

有標籤體素的類別占比：floor 36 %、wall 35 %、ceiling 18 %，其次是 windowpane、door、column、shelf、armchair、chair、sofa、counter、light；`person` 0.3 %（1457 個體素，即 `clean_map` 從 `map.pcd` 中移除的殘影；遮罩版本繼承了這個移除，原始融合結果則沒有）；`stairway` 801、`escalator` 990。

這對計畫的意義：

- 這套做法如設計般可行，`pgo_node` 完全不需要修改。存檔唯一缺少的是關鍵幀時間戳：在 `poses.txt` 旁放一個 `stamps.txt`（或者加一欄，前提是先確認後端的讀取器）就能取代重播時的記錄。
- 沒有跑 Grounded-SAM 2：第一個目標是封閉集合的 stuff 類別，Mask2Former 已涵蓋。標註器可接受任何輸出相同 `(label, confidence, boundary)` 的 `Segmenter`，所以物體實例那一輪只是換個後端。
- 在模型之前，外參才是瓶頸：先校正（`direct_visual_lidar_calibration`）再評估物體類別，並持續保留疊圖檢查。
- 只有前方視錐被標註，所以地圖有三分之二維持未標註；就覆蓋率而言，更廣角的鏡頭或第二台相機比更好的模型更重要。
- 融合是以關鍵幀密度（0.5 m / 10°）逐 patch 進行；若標註關鍵幀之間的每一次掃描，每個體素的票數會倍增，但分割成本是 10 倍。

## 2D 後端比較（2026-10-17，`dp1f_with_image`）

同一次重播（這次是 1066 個關鍵幀——即時重播無法逐位元重現——13 個迴環、8.2 M 點，`clean_map` 在 84 秒內 `ok`）、同樣的關鍵幀時間戳、投影與融合，只換掉 2D 模型（`tools/semantic_offline/segment.py`，每個後端一個類別）。各次執行放在 `map/dp1f_with_image/semantic/<tag>/`，表格以及把每次執行的俯視圖與 QA 疊圖並排的網頁在 `semantic/compare/compare.{md,html}`（原始融合）和 `semantic/compare_clean/`（以清理後的 `map.pcd` 遮罩）。

沒有人工標註的 ground truth，所以 `compare.py` 用兩樣東西來評分：地圖自身的幾何（相對於局部地板的高度——最近關鍵幀的 z 減去 0.481 m 的 LiDAR 高度——以及體素法向量），以及 Swin-L 參考執行；比較前先用 `taxonomy.py` 把每種標籤空間折成 16 個粗類別。「floor low+flat」是被標為 floor 的體素中，距地板 0.25 m 內且法向量近乎垂直者的比例；「geo-floor → floor」是具有地板幾何且有標籤的體素中，拿到 floor 標籤者的比例。天花板的比率刻意偏弱（風管、樑和夾層都會算進去），只能在各列之間比較。時間是這台 Mac 的 MPS 上每張 1280×720 影像的秒數，包含放大回全解析度與邊界遮罩；請當作比值來用。

| 執行 | 秒 / 張 | 有標籤 | 純度 | floor low+flat | geo-floor → floor | wall vertical | 與 Swin-L 一致 |
|---|---|---|---|---|---|---|---|
| Mask2Former Swin-L ADE20K（參考） | 0.96 | 31 % | 0.95 | 80 % | 98 % | 82 % | – |
| Mask2Former Swin-T ADE20K | 0.53 | 31 % | 0.95 | 81 % | 97 % | 80 % | 88 % |
| SegFormer-B5 ADE20K 640 | 0.62 | 32 % | 0.96 | 80 % | 98 % | 82 % | 89 % |
| SegFormer-B2 ADE20K 512 | 0.40 | 32 % | 0.96 | 79 % | 98 % | 81 % | 87 % |
| OneFormer Swin-L ADE20K | 1.27 | 32 % | 0.95 | 81 % | 97 % | 81 % | 92 % |
| Mask2Former Swin-L COCO panoptic | 1.29 | 32 % | 0.93 | 81 % | 97 % | 80 % | 83 % |
| Grounding DINO tiny + SAM ViT-B（18 個提示詞，每 3 個關鍵幀取 1 個） | 3.45 | 8 % | 0.97 | 84 % | 99 % | 96 % | 90 % |

有標籤體素的粗類別占比，原始融合（%）：

| 執行 | floor | wall | ceiling | door | person | seat | table | column | window | storage | other |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Swin-L ADE | 36.5 | 34.7 | 18.3 | 0.9 | 0.4 | 2.0 | 0.8 | 0.9 | 2.2 | 1.2 | 0.9 |
| Swin-T ADE | 36.1 | 37.9 | 17.5 | 0.8 | 0.3 | 1.3 | 0.4 | 1.5 | 2.2 | 0.2 | 0.5 |
| SegFormer-B5 | 36.7 | 36.1 | 20.4 | 0.6 | 0.4 | 1.4 | 0.3 | 0.6 | 2.3 | 0.2 | 0.3 |
| SegFormer-B2 | 37.0 | 36.5 | 21.2 | 0.5 | 0.4 | 1.2 | 0.1 | 0.3 | 1.7 | 0.1 | 0.3 |
| OneFormer Swin-L | 35.9 | 35.1 | 17.3 | 1.2 | 0.3 | 2.2 | 1.6 | 1.3 | 1.9 | 0.4 | 0.9 |
| Swin-L COCO | 35.3 | 30.2 | 16.0 | 1.7 | 0.4 | 3.5 | 0.7 | 0.0 | 5.4 | 0.8 | 3.7 |
| Grounded-SAM | 75.0 | 6.3 | 6.3 | 1.7 | 0.9 | 3.1 | 0.5 | 1.5 | 1.6 | 0.9 | 0.0 |

數字說明了什麼：

- **在可通行性類別上，模型是哪一個並不重要。** 每個 ADE20K 模型在 floor / wall / ceiling 上都落在彼此兩個百分點之內，覆蓋率相同（每個 patch 的 16 %、體素的 31 %——決定它的是視錐，不是模型），純度也相同。地圖的地板層（`view_topdown_lowband*.jpg`）在它們之間可以互換。
- **模型的差異在物體和遮罩邊緣。** Swin-L 和 OneFormer 找到的 seat / table / storage / column 體素大約是 SegFormer-B2 的兩倍（2.0 / 0.8 / 1.2 / 0.9 % 對 1.2 / 0.1 / 0.1 / 0.3 %），而且 B2 的遮罩較糊：在 QA 疊圖中地板會沿著人的腳往上滲，天花板 / 牆的邊界也會漂移。與參考執行的體素不一致（ADE 模型為 8–13 %）幾乎全部集中在這裡；`compare.json` 中的 reference-confusion 顯示 floor 一致率 97–99 %、wall 86–92 %、ceiling 88–95 %。
- **SegFormer-B2 是上機器人的候選。** 在這裡比 Swin-L 快 2.4 倍（而且是 27 M 參數的模型，TensorRT 在 Jetson 上處理得來），換到的 floor / wall / ceiling 層是一樣的。當需要物體標籤時，存檔後再從 patches 離線重跑大模型。
- **COCO panoptic 在這個場地沒有任何好處。** 它把玻璃帷幕讀成 `window`（5.4 %），有 3.7 % 落在分類法之外的類別，和參考執行的一致率最低（83 %）；相對參考執行的 wall 精確率為 77 %。ADE20K 的 150 類已涵蓋地圖需要的一切（floor、wall、stairs、escalator、door、person、column）。
- **Grounded-SAM 是一道物體處理，不是語意模型。** 框對 stuff 來說是錯的基本單位：只有 8 % 的體素拿到標籤，其中 75 % 是 floor；牆和天花板幾乎沒有標籤（它標到的少數是對的，所以 wall 比率有 96 %）。它確實找得到座椅、門、樓梯和人（3.1 / 1.7 / 1.0 / 0.9 %），且是七者中逐物體遮罩最乾淨的，但每張影像 3.5 秒——約為封閉集合模型的 10 倍。若需要物體實例，應該在 ADE20K 的 stuff 地圖之上把它當作第二道處理，而不是取而代之；屆時該試的是 SAM 3 和單趟的 EOVSAM（見參考資料）。
- **管線本身的限制仍然大過模型的限制。** 覆蓋率（31 % 的體素有標籤）、手量的外參，以及每個模型都帶進原始融合的約 1500 個 `person` 殘影體素（遮罩版本會減半，因為 `clean_map` 已經移除那些點），對結果的影響都大過換任何一個後端。模型選擇暫時定案：接下來是覆蓋率與校正，然後是關鍵幀時間戳的 sidecar，好讓重播時的記錄可以退場。

這次執行也暴露了工具上的兩件事：`poses.txt` 仍然沒有時間戳，所以重播記錄還是必要的；另外，`build/` 被刪掉的工作區會讓 `install/syncai_common` 變成懸空的 `--symlink-install` 連結——C++ node 照常執行，但 `ros2 service call` 無法匯入服務型別（`replay.sh` 現在會在播放前先檢查；把該套件重建到一個暫存 prefix 就夠了）。

## 物體地圖（2026-10-18，`dp1f_with_image`）

語意體素回答的是「這裡是什麼」；「綠色沙發在哪」需要的是物體。在同一次重播上試了兩種做法，最後都產出同樣的 `objects.json`（類別、顏色詞、最多三張裁圖、裁圖的 SigLIP 2 向量，以及距物體 0.6–1.5 m、面向物體的可站地板 pose），也用同一個文字查詢（`query_objects.py`，中英文皆可，以物體向量的 cosine 排序）。輸出：`semantic/objects/` 與 `semantic/instances_grounded-sam/`。

| | 分群（`objects.py`） | 遮罩 + 關聯（`instances.py`） |
|---|---|---|
| 來源 | 對 Swin-L 地圖的「物體」體素做 DBSCAN | 每 2 個關鍵幀跑 Grounding DINO + SAM 遮罩，投影後關聯 |
| 時間 | 語意地圖之外約 2 分鐘 | 分割 18 分鐘（每張 2.0 秒），從快取重跑關聯只要幾秒 |
| 物體數 | 114 | 188 |
| 青綠沙發區：沙發 / 單人沙發框數（地圖上約 18–21 件） | 26，數張沙發被框兩次 | 20，框較窄（只涵蓋看到的面），漏掉幾張 |
| 裁圖 | 仰拍，常只有半張沙發 | 每張裁圖是一個完整物體（遮罩的框） |
| 誤判 | 少（ADE 類別對家具可靠） | 多：「escalator」打在走廊、玻璃門和樓梯上，「armchair」打在吧檯椅和餐椅上 |
| 小東西 | 沒有（低於體素門檻） | 找到滅火器、垃圾桶、類似 ATM 的機器 |

兩者查「綠色沙發」/「green sofa」都回傳落地窗邊的深青綠沙發，查「grey leather armchair」都回傳北側大廳的灰褐皮沙發。顏色詞不可靠（青綠被標成 `cyan` 或 `black`，灰褐被標成 `green`；白平衡是鎖死的），向量則可靠。請以 cosine 排序：加了提示詞模板之後，SigLIP 的 sigmoid 對同類物體都接近 1，所以「找不到」需要另外的規則（與第二名的分差，或類別 + 向量）。

關聯的教訓，每一條都來自一次失敗的執行：共享 0.2 m 體素會配對失敗（不同視角打到沙發的不同面）；沒有上限的外框規則會把整個沙發區串成 5–7 m 的「沙發」；「接近」要在地板平面上量；而一個遮罩的約 20 個 LiDAR 點裡，總有幾個穿過縫隙打到後方物體（穩健外框 1.4 m、完整外框 2.4 m、p90 4.4 m），所以每個片段要先只留最大的 3D 群。之後外框尺寸就合理了（中位數 1.1 m）。

當時的瓶頸是偵測器，不是關聯：上面的誤判都是 Grounding DINO 的標籤。**SAM 3**（`--backend sam3`，2026-10-18 取得存取權後執行）解決了這個問題，用的是同樣的 533 張影像、提示詞和關聯參數（`semantic/instances_sam3/`）：

| | Grounding DINO + SAM | SAM 3 | SAM 3，`--min-obs 4` |
|---|---|---|---|
| 秒 / 張（MPS，18 個提示詞） | 2.0 | 10.9 | （同一批片段） |
| 片段 / 保留的物體 | 985 / 188 | 1332 / 189 | 1332 / 101 |
| 「escalator」物體 | 21 個，幾乎都是走廊 / 玻璃門 | 1 個（從另一側看到的樓梯） | 0 |
| 青綠沙發區的沙發 + 椅子框數（實際約 18–21 件） | 20 | 33 | 21 |
| 小物體（滅火器、垃圾桶、盆栽、ATM） | 有一些，混著誤判 | 全部找到，裁圖正確 | 大部分（少了一台類 ATM 機器和一盆盆栽） |

用 SAM 3 時，圖庫裡每張裁圖都是標籤所說的東西；ATM 被標成 `vending machine`（最接近的提示詞），樓梯被標成 `stairs`。物體被看到的次數多得多（沙發通常 20–34 次，Grounding DINO 是 2–10 次），所以 `--min-obs` 成了可用的過濾器：預設門檻 2 時沙發區多出來的框，都是已找到的沙發只被看到 2–3 次的細長碎片。查詢（「綠色沙發」、「grey leather armchair」、「fire extinguisher」、「trash can」、「stairs」）都回傳正確物體。代價是時間：每個提示詞跑一次，是 Grounding DINO 的 5.4 倍；離線跑這個 bag 約 95 分鐘。**實例後端採用 SAM 3，搭配 `--min-obs 3–4`**；要加速就把提示詞清單縮到場地真正需要的類別。

## 參考資料

- [gfchen01/semantic_mapping_with_360_camera_and_3d_lidar](https://github.com/gfchen01/semantic_mapping_with_360_camera_and_3d_lidar)：ROS 2 node，影像 + 3D LiDAR + 里程計，Grounded-SAM-2，物體層級地圖，附範例 bag。最接近的現有系統。
- [A Comparison of Segmentation Methods for Semantic OctoMap Generation (2025)](https://www.doi.org/10.3390/app15137285)：ROS 2 標註寫入 `octomap_server`；記錄了偽色混合的問題。
- [Camera-Lidar Integration: Probabilistic sensor fusion for semantic mapping (2020)](https://arxiv.org/pdf/2007.05490)：每個體素對類別做離散貝氏濾波。
- [Fusing Lidar and Semantic Image Information in Octree Maps (ACRA 2017)](https://www.araa.asn.au/acra/acra2017/papers/pap109s1-file1.pdf)：投影 + Octomap，降低靠近遮罩邊界之點的權重。
- [Incremental Semantics-Aided Meshing from LiDAR-Inertial Odometry and RGB Direct Label Transfer (2026)](https://arxiv.org/pdf/2604.09478)：以 FAST-LIO2 為骨幹，逐體素標籤直方圖，無迴環閉合。
- [FindAnything (2025)](https://arxiv.org/pdf/2504.08603)、[OVI-MAP (2026)](https://arxiv.org/pdf/2603.26541)、[LOSC (2025)](https://arxiv.org/pdf/2507.07605)：開放詞彙物體地圖與 LiDAR 開放詞彙分割；研究用程式碼，ROS 2 支援未經驗證。
- [CLIP2Scene (CVPR 2023)](https://arxiv.org/pdf/2301.04926.pdf)：2D 到 3D 的 CLIP 蒸餾；推論時只需點雲，但需要在我們的 LiDAR 與場景上重新預訓練。
- [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)：從 bag 做 LiDAR-相機外參校正。
- [SAM 3: Segment Anything with Concepts](https://github.com/facebookresearch/sam3) 與 [EOVSAM (2026)](https://www.alphaxiv.org/abs/2608.02284)：文字提示的實例分割及其單趟語意版本——Grounded-SAM 之後該試的開放詞彙後端（每個概念一趟推論正是後者比封閉集合模型慢約 10 倍的原因）。
- [FUS3DMaps (2026)](https://arxiv.org/pdf/2605.03669)、[OpenVox (2025)](https://arxiv.org/html/2502.16528v1)、[SLAMVDB](https://robogroup.edi.lv/technology/slamvdb)：同時有稠密層與實例層的開放詞彙體素地圖（最後一個是每個體素帶視覺語言嵌入的 ROS 2 套件）——若建圖後的文字查詢成為需求，這條管線會往這裡走。
