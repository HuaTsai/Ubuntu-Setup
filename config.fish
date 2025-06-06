if status is-interactive
    fortune | cowsay | lolcat
end

set -gx fish_greeting
set -gx PATH /usr/src/tensorrt/bin /usr/local/cuda/bin $PATH

bass source /opt/ros/rolling/setup.bash
alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
alias crm='rm -rf build install log'
alias sis='source install/setup.bash'
alias gst='git status -sb'
alias glo='git log --oneline --graph --decorate --all'
alias cat='batcat -pp'
alias bat='batcat'

function grn
    grep -rn --color=auto --exclude-dir={.git,node_modules,__pycache__} $argv .
end

function fish_command_not_found
    set RESPONSES \
        "你是不是手殘了？🤡" \
        "這條指令不存在，你在做夢？💀" \
        "系統拒絕執行這條智障指令 🚫" \
        "你打算讓電腦爆炸嗎？💥" \
        "連這都打錯，還是別用終端了！😂" \
        "別亂打指令，小心 rm -rf / 😈" \
        "你確定這不是 Windows 指令？🤣" \
        "這條指令不存在，快去學 Linux！🤣" \
        "又打錯了？你該換個鍵盤了吧！😏" \
        "這是 Linux 不是 Windows，別亂試 🖥️" \
        "你是鍵盤亂打一族嗎？🤔" \
        "電腦：你再這樣，我就自爆了！💣" \
        "這條指令是什麼黑魔法？👀" \
        "你是不是剛從 Windows 轉過來？😂" \
        "錯誤的指令 + 錯誤的使用者 = 大災難 🚨" \
        "再試一次？還是先去 Google 吧 📖" \
        "你是不是打算召喚惡魔？😈" \
        "這條指令只有 AI 能理解 🧠" \
        "這指令是你亂編的吧？😂" \
        "這種東西不存在，你腦補的？🤯" \
        "不會用終端？那就別亂玩！🤣" \
        "輸入 sudo rm -rf / 來解決一切問題 😈（開玩笑的，別真的做）" \
        "Linux 給了你力量，但你還不會用 🧙" \
        "你的終端正在嘲笑你... 🤡" \
        "這條指令來自異世界？🌍" \
        "這指令還沒被發明呢，等你去開發吧 🤖" \
        "你可以繼續輸入，反正還是錯的 😏" \
        "這指令不存在，但你的錯誤已經超神了 🤦" \
        "看來你需要 man google 來拯救自己 🔍" \
        "我猜你在敲鍵盤時沒帶腦 🤯" \
        "你不如試試 help，或者直接投降 🙃" \
        "不要再亂打了，我已經在替你尷尬了 😵" \
        "你的電腦已經開始懷疑你的智商了 🤔" \
        "要不要考慮回去用 GUI？🖥️" \
        "這是 AI 能懂的語言嗎？👽" \
        "你是不是喝醉了？這指令長得好奇怪 🍺" \
        "這條指令不存在，但你的勇氣可嘉 💪" \
        "你剛剛是不是隨機按鍵盤？🤨" \
        "這樣輸入，電腦真的會動嗎？🤔" \
        "拜託，你至少 Google 一下吧 🤦" \
        "你的 Linux 教師已經放棄你了 📚" \
        "這指令是從哪裡學來的？黑魔法？🧙" \
        "這是終端機，不是你的日記本 📖" \
        "連這個都不會，我該報警了嗎？🚨" \
        "你是不是在亂試？終端機會哭的 😭" \
        "你連這都不會，還想當駭客？💻" \
        "這條指令沒有作用，但你可以繼續試試 🤡" \
        "請稍等，我幫你叫個技術支援 📞" \
        "我懷疑你在打密碼，還是密碼比較合理 🤣" \
        "你這樣打指令，連 ChatGPT 也救不了你 💀"
    set RAND (math 1 + (random) % (count $RESPONSES))
    echo $RESPONSES[$RAND] | cowsay -f dragon | lolcat
end
