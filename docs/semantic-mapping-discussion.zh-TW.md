# 語意建圖：討論紀錄（2026-10-17 / 18）

> 本文為 [`semantic-mapping-discussion.md`](semantic-mapping-discussion.md) 的繁體中文翻譯。英文版為正本，修改時請先改英文版，再同步本檔；若兩者不一致，以英文版為準。

建置離線語意地圖管線時提出的問題，以及實驗得出的答案。每項主張背後的量測都在 [`semantic-mapping.md`](semantic-mapping.md)（「2D backend comparison」、「Object map」）；工具與指令在 `tools/semantic_offline/README.md`。本檔記錄的是工作*為什麼*這樣走，讓下一個決定可以從這裡開始。

## 1. 相機影像能把 LiDAR 地圖變成語意地圖嗎？

可以，而且不用改 `pgo_node`。2D 模型標註每個關鍵幀最近的影像，標籤投影到該關鍵幀的 patch 上（位姿內插到影像時間戳，用 z-buffer 處理遮擋），再用迴環修正後的 `poses.txt` 逐 0.1 m 體素融合（類別直方圖、取 argmax）。在 `record/dp1f_with_image` 上端到端跑過：31 % 的體素有標籤，投票純度 0.95。把標籤附在關鍵幀而不是世界座標的體素上，是迴環修正後仍能對齊的關鍵。

存檔沒有提供、因此要付出的代價：`poses.txt` 沒有時間戳，所以重播時必須錄下 `lio_odom` 和 pgo 的 `map → pointlio_odom` TF 來還原。在 `poses.txt` 旁邊放一個 `stamps.txt` 就能省掉這一步。

## 2. 語意地圖拿來做什麼？

依「價值 / 成本」排序，並列出各自需要什麼：

1. **從地圖產生 keepout mask。** 樓梯、手扶梯、玻璃帷幕：Mid360 會穿透玻璃，所以雷射地圖把玻璃牆當成可走，相機卻標得出 `window`。投影到 2D、寫成 `keepout.yaml` / `.pgm`，就能透過現有的 `filter_mask_server/load_map` 路徑送到 planner。離線、不改執行期、補上一個安全缺口。**建議先做。**
2. **gridmap 轉換方法的選擇。** backend 有兩套 pcd → gridmap 方法，刻意不自動選擇，因為缺少可靠的依據。語意地板層正是那個依據。
3. **預填 octomap 的地板編輯。** console 的地面編輯器要先手動標地板再跑 `apply_octomap_edits`；亮面地板會留下空洞和反射。語意 `floor` 可以預填 `octomap_edits.json`，讓操作員只需修正。
4. **`clean_map` 直接丟掉 `person` 點**，而不只靠 miss 比例。
5. **語意目標**（「去門口」「去綠色沙發」）：需要的是物體，不是體素。見第 4 節。
6. **線上應用**（人旁邊減速、localizer 不用動態點對位、為規劃中的充電座 BT 偵測充電座）：需要在 Jetson 上跑模型，成本最高。

## 3. 這麼多 2D 模型，選哪個？

依「在哪裡跑、要哪些類別」來選，不要看排行榜。七個後端都跑過完全相同的投影與融合；在 floor / wall / ceiling 上，每個 ADE20K 模型都落在彼此兩個百分點內，所以對可通行性來說選哪個沒差。差別在物體和遮罩邊緣。

| 用途 | 選擇 | 理由 |
|---|---|---|
| 存檔後離線 | Mask2Former Swin-L ADE20K | 地板層相同，物體體素約為小模型的兩倍；離線不在乎延遲 |
| Jetson 上即時 | SegFormer-B2 ADE20K | 這裡快 2.4 倍，地板層相同；物體較弱、邊緣較糊。Jetson / TensorRT 延遲尚未量測 |
| 物體、自由文字名稱 | SAM 3（未取得前用 Grounding DINO + SAM） | 疊在 ADE stuff 地圖上的第二道，不取代它 |
| 不採用 | COCO panoptic、OneFormer | COCO 誤讀玻璃帷幕、一致率最低；OneFormer 比 Swin-L 慢 30 %，沒有看得到的好處 |

這個答案有兩個限制。物體類別還沒有真正的評分：比較是拿地圖自身的幾何當 ground truth，只涵蓋 floor / wall / ceiling；要比較門、玻璃、樓梯需要約 50 張人工標註影像，只有開始做第 2 節的第 1 或第 6 項時才值得。另外，管線本身的限制比換模型影響更大：只有前方視錐被標註（31 % 的體素），相機外參還是 URDF 手量值。覆蓋率與校正要排在調模型之前。

## 4. 標記物體：「綠色沙發在哪？」

「綠色沙發」是類別（ADE 有 `sofa`）加上屬性（顏色，體素地圖沒有存），而「在哪」需要把體素組成物體。計畫分三階段：

1. **用分群產生實例**：對現有語意地圖的「物體」體素，依粗類別分群。
2. **每個物體的屬性**：挑兩三個最佳視角（視錐、z-buffer、距離、不被影像邊緣切到），各裁一張圖，從物體自身體素落點的像素取顏色詞，並算一個 SigLIP 2 向量（裁圖的平均）。文字查詢就是對物體向量算 cosine；SigLIP 2 是多語模型，「綠色沙發」直接打就能用。用每個物體的裁圖而不是逐像素 CLIP 特徵：查詢只需要排序物體，一張地圖幾百次影像編碼很便宜。
3. **用 2D 遮罩產生實例**（SAM 3），在修正後的世界座標中跨關鍵幀關聯，取代分群。

每個物體還會得到一個距離 0.6–1.5 m、面向物體的可站地板 pose，讓查到的結果可以直接當 MOVE 目標。輸出的 `objects.json` 遵守地圖目錄的約定：不含地圖名稱、不含絕對路徑。

跑出來的結果：

- 第 1 + 2 階段查「green sofa」回傳落地窗邊的深青綠沙發，查「grey leather armchair」回傳北側大廳的灰褐沙發，語意地圖之外只多約兩分鐘。分群會碎片化：幾個沙發區就有 42 個沙發 / 單人沙發實例。
- 顏色詞不可靠（青綠被標成 `cyan` / `black`，灰褐被標成 `green`；相機白平衡是鎖死的）；向量則能正確區分。查詢靠向量，顏色詞只當參考。
- 分數不能當門檻：加了提示詞模板後，SigLIP 的 sigmoid 對同類物體都接近 1。「找不到」需要自己的規則（與第二名的分差，或類別 + 向量）。
- 第 3 階段先用 Grounding DINO + SAM 代替 SAM 3：裁圖都是完整物體，沙發區從 26 個框降到 20 個，地圖上實際約 18–21 件，也出現了小物體（滅火器、垃圾桶）。過程中關聯試了四次，每次失敗都有收穫：共享體素會配對失敗，因為不同視角打到不同的面；沒有上限的外框規則會把整個沙發區串成一張 7 m 的「沙發」；「接近」要在地板平面上量；每個遮罩的點要先只留最大的 3D 群，因為總有幾個點穿過縫隙打到後方。
- 第 3 階段現在的瓶頸是偵測器：Grounding DINO 把走廊和玻璃門叫成「escalator」，把吧檯椅叫成「armchair」。這正是 SAM 3 要解決的。

## 5. SAM 3 現況

後端已依 transformers 5.20 的 API 寫好（共用影像特徵、每個提示詞一次 forward），並對照已安裝的原始碼確認過，但從未實際跑過：`facebook/sam3` 和 `facebook/sam3.1` 都需要人工核准，2026-10-18 存取申請仍在等待審核（Hugging Face 登入本身沒問題）。沒有使用非官方轉存的權重。核准後：

```
.venv/bin/python instances.py <map dir> <bag dir> <keyframes.txt> <lio_odom.txt> <out dir> --backend sam3
```

先用一張影像確認；再和 `instances_grounded-sam/` 比較誤判、沙發區數量和同一組查詢。

## 6. 待辦

- SAM 3 執行與比較（等待存取核准）。
- 物體查詢的「找不到」規則。
- 從 `map_semantic_clean.pcd` 產生 keepout mask（第 2 節第 1 項），並在這張地圖的玻璃帷幕和手扶梯上檢查。
- 把某個物體的目標 pose 送給機器人或模擬環境的 `navigate_to_goal`，看停的位置是否合理；離線無法驗證。
- `save_maps` 在 `poses.txt` 旁輸出 `stamps.txt`。
- 評估細小物體前先做相機外參校正（`direct_visual_lidar_calibration`）。
- Workspace 的 `build/` 在 2026-10-17 執行中途消失，`install/` 變成懸空的 symlink；Workspace 需要重建（重播時用的是 `syncai_common` 和 `clean_map` 的暫存建置）。
