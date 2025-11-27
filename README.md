# Sokoban Solver - 平行程式設計作業

這是一個使用 C++ 實作的 Sokoban 遊戲求解器，採用平行化 A\* 搜尋演算法。

### 核心程式

- `hw1.cpp` - 主要求解器程式，使用反向搜尋和平行化 A\* 演算法
- `hw1_forward.cpp` - 前向搜尋版本的求解器（備用實作）
- `Makefile` - 編譯設定檔

### 測試與驗證

- `samples/` - 測試案例資料夾（01.txt ~ 25.txt）
- `test_all.sh` - 自動化測試腳本
- `validate.py` - 解答驗證程式
- `example_solver.py` -   課堂提供 Python 參考實作
- `play.py` - 互動式遊戲介面

## 環境要求

### 必要套件

- **編譯器**: g++ (支援 C++17)
- **函式庫**:
  - Intel TBB (Threading Building Blocks)
  - OpenMP
  - Boost (用於 hash 函式)

### Linux 安裝指令

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential libtbb-dev libomp-dev libboost-dev

# CentOS/RHEL
sudo yum install gcc-c++ tbb-devel libomp-devel boost-devel
```

## 編譯與執行

### 編譯程式

```bash
make
```

### 執行求解器

```bash
./hw1 samples/01.txt
```

### 執行所有測試

```bash
bash test_all.sh
```

### 驗證單一解答

```bash
python3 validate.py samples/01.txt answer.txt
```

## 演算法特色

- **反向搜尋**: 從目標狀態往初始狀態搜尋，提高效率
- **平行化處理**: 使用 OpenMP 進行多執行緒搜尋
- **狀態正規化**: 減少搜尋空間，避免重複狀態
- **啟發式函數**: 使用曼哈頓距離計算 h-cost
- **並行資料結構**: 使用 TBB 的 concurrent 容器

## 輸入格式

推箱子地圖使用以下符號：

- `#` - 牆壁
- ` ` - 空地
- `.` - 目標位置
- `o` - 玩家
- `x` - 箱子
- `X` - 箱子在目標位置上
- `O` - 玩家在目標位置上

## 輸出格式

輸出移動序列，使用以下字符：

- `S` - 向下移動
- `W` - 向上移動
- `A` - 向左移動
- `D` - 向右移動

## 效能

- 支援最大 256x256 的地圖
- 使用 6 個執行緒進行平行搜尋
- 針對複雜關卡進行最佳化

## 注意事項

- 程式會自動設定 10 秒執行時間限制
- 如果無解會輸出錯誤訊息
- 建議在多核心 CPU 上執行以獲得最佳效能
