#!/bin/bash

# 編譯程式
echo "編譯 hw1.cpp..."
make
if [ $? -ne 0 ]; then
    echo "編譯失敗！"
    exit 1
fi

echo "開始測試所有 test cases..."
echo "================================"

passed=0
failed=0

# 測試所有 25 個 test cases
for i in $(seq -f "%02g" 1 25); do
    input_file="samples/${i}.txt"
    answer_file="answer_${i}.txt"
    
    if [ ! -f "$input_file" ]; then
        echo "Test case ${i}: 檔案 $input_file 不存在，跳過"
        continue
    fi
    
    echo -n "Test case ${i}: "
    
    # 執行程式並將輸出存到答案檔案（3秒timeout）
    timeout 10 ./hw1 "$input_file" > "$answer_file" 2>/dev/null
    exit_code=$?
    
    if [ $exit_code -eq 124 ]; then
        echo "超時 (>3秒) ⏰"
        ((failed++))
        rm -f "$answer_file"
        continue
    elif [ $exit_code -ne 0 ]; then
        echo "執行失敗"
        ((failed++))
        rm -f "$answer_file"
        continue
    fi
    
    # 驗證答案
    python3 validate.py "$input_file" "$answer_file" > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo "通過 ✓"
        ((passed++))
    else
        echo "失敗 ✗"
        ((failed++))
    fi
    
    # 清理答案檔案
    rm -f "$answer_file"
done

echo "================================"
echo "測試完成！"
echo "通過: $passed"
echo "失敗: $failed"
echo "總計: $((passed + failed))"

if [ $failed -eq 0 ]; then
    echo "🎉 所有測試都通過了！"
    exit 0
else
    echo "❌ 有 $failed 個測試失敗"
    exit 1
fi
