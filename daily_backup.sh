#!/bin/bash

# Configuration
log_file="WORK_LOG.md"
current_date=$(date "+%Y-%m-%d")

# Prompt for work description
echo "오늘 수행한 작업 내용을 입력하세요 (엔터를 치면 입력이 종료됩니다):"
read -r work_description

if [ -z "$work_description" ]; then
    echo "작업 내용이 입력되지 않아 스크립트를 종료합니다."
    exit 1
fi

# Append to WORK_LOG.md
# Check if the date header already exists for today
if ! grep -q "## $current_date" "$log_file"; then
    echo -e "\n## $current_date" >> "$log_file"
fi
echo "- $work_description" >> "$log_file"
echo "WORK_LOG.md 파일이 업데이트되었습니다."

# Git operations
echo "Git에 변경 사항을 추가하고 커밋합니다..."
git add .
git commit -m "Daily update: $current_date - $work_description"

echo "Github으로 푸시합니다..."
# Assumes 'origin' remote is set up. You might need to change 'main' to 'master' if using older git defaults.
git push origin main

echo "완료되었습니다!"
