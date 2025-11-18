# Ubuntu-Setup

## Installation

1. Download the [Ubuntu ISO image](https://releases.ubuntu.com)
2. Create a bootable USB flash drive via Ubuntu suggested tool [balenaEtcher](https://etcher.balena.io/)
3. Follow the steps to install

### Recover the USB Drive

- After creating the drive, it may be no longer accessed by windows normally.
- To recover the drive, try following commands

  ```powershell
  diskpart.exe
  list disk
  select disk <N>
  clean
  create partition primary
  active
  list partition
  select partition <N>
  format FS=NTFS override quick
  exit
  ```

## Setup

### Initial Setups (Ubuntu 24.04)

- Dual system time synchronization
  - `sudo timedatectl set-local-rtc 1`
- Settings
  - Multitasking
    - Hot Corner: On
  - Ubuntu Desktop
    - Auto-hide the Dock: On
  - System
    - Region & Language > System > Manage Installed Languages
      - Install / Remove Languages > Chinese (traditional)
      - Logout and then login
  - Keyboard
    - Input Source > Add Input Source... > Chinese (Taiwan) > Chinese (Chewing) > Add
- Dock apps
  - Browser
  - Files
  - Terminal
    - Install Font `CaskaydiaCoveNerdFontMono-Regular.ttf`
    - Preferences > Unnamed
      - Custom font: Font CaskaydiaCove Nerd Font Mono
    - Size 22
    - Colors (TokyoNight)
      - Foreground color: #a9b1d6
      - Background color: #1a1b26
    - Logout and then login
    - Reference: [Nerd font](https://www.nerdfonts.com/font-downloads)
  - Visual Studio Code
    - Download from [official website](https://code.visualstudio.com/)

### Basic Tools

```bash
sudo apt update && sudo apt upgrade
sudo apt install aptitude btop build-essential curl dos2unix git htop hyperfine libfuse2 locate neofetch net-tools openssh-server python3-pip python3-venv software-properties-common tldr tree
```

### Fish Shell

```bash
sudo apt-add-repository ppa:fish-shell/release-3
sudo apt update
sudo apt install fish
chsh -s $(which fish)
# Logout and then login
curl -fsSL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher
fisher install jethrokuan/z  # jump command z
fisher install edc/bass      # support source command
echo "set -gx fish_greeting" >> ~/.config/fish/config.fish
```

### SSD

- Install new SSD

  ```bash
  sudo fdisk -l  # Check disk, it should appear without partition
  sudo fdisk /dev/nvme1n1  # Create partitions, use default values and finally write it
  sudo mkfs -t ext4 -c /dev/nvme1n1p1  # Then it should appears
  ```

- Access configured SSD

  If SSD is not shown in the dock, we can run the following commands

  ```bash
  $ sudo fdisk -l  # Check target disk
  $ sudo blkid /dev/nvme1n1p1  # Show disk UUID
  $ sudo mkdir /media/SSD && sudo chown <user>:<user> /media/SSD  # Create directory and change owner
  $ cat /etc/fstab  # Add mount directory
  ...
  UUID=... /media/SSD ext4 defaults 0 0
  ...
  ```

### Bat instead of cat

- Run `sudo apt install bat`
- Fish shell add aliases

  ```bash
  alias cat='batcat -pp'
  alias bat='batcat'
  ```

### Command grep alias

- Fish shell add

  ```cpp
  function grn
      grep -rn --color=auto --exclude-dir={.git,node_modules,__pycache__} $argv .
  end
  ```

### Multiple `g++` Versions

For upstream versions, add software sources [ToolChain](https://wiki.ubuntu.com/ToolChain) via ppa.

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
```

Assume we want to select choose `g++` from `g++-13` and `g++-14`.

```bash
# <path/to/symlink> <command_name> <path/to/command_binary> <priority>
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 90
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 90
```

Default g++ version will be `g++-14` since it has higher priority.
To configure g++ version, run `sudo update-alternatives --config g++`

### SSH

```bash
$ ssh-keygen
Generating public/private ed25519 key pair.
Enter file in which to save the key (/home/<user>/.ssh/id_ed25519):
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in /home/<user>/.ssh/id_ed25519
Your public key has been saved in /home/<user>/.ssh/id_ed25519.pub
The key fingerprint is:
SHA256:EB4RQsFiUmkNaAQhgAMKCqN3RVu5TRg0kwQ4ehYvBWU <user>@<user>
The key's randomart image is:
+--[ED25519 256]--+
|/=o*=*E*B=       |
|%o= *=.=+o.      |
|==.o.=+  +       |
| ...+ ... .      |
|   o .  S        |
|                 |
|                 |
|                 |
|                 |
+----[SHA256]-----+
$ cat ~/.ssh/id_ed25519.pub  # SSH public key
$ ssh-copy-id user@remote-ip  # type password
$ cat ~/.ssh/config
Host nickname
  User user
  Hostname remote-ip
$ ssh nickname  # connect to remote machine
```

### Git / Github

- Git

  ```bash
  git config --global init.defaultBranch main
  git config --global user.name "HuaTsai"
  git config --global user.email "huatsai42@gmail.com"
  ```

- SSH key

  ```bash
  $ cat ~/.ssh/id_rsa.pub
  # Copy the output public key to github settings
  ```

- GPG key

  ```bash
  $ gpg --full-generate-key
  # Default options:
  #   (9) ECC (sign and encrypt)
  #   (1) Curve 25519
  #   0 = key does not expire
  #   Real name: HuaTsai
  #   Email address: huatsai42@gmail.com
  $ gpg -k  # list public keys, use -K to list private keys
  /home/<user>/.gnupg/pubring.kbx
  --------------------------------
  pub   ed25519 2025-06-05 [SC]  # [SC] means sign and certify
        112BEF7089D486F1770B8A0E9AADD83172BA05B2  # fingerprint
  uid           [ultimate] HuaTsai <huatsai42@gmail.com>
  sub   cv25519 2025-06-05 [E]   # [E] Means encryption (subkey)
  $ gpg -a --export 112BEF7089D486F1770B8A0E9AADD83172BA05B2
  # Export ASCII public key, copy the key to github settings
  $ git config --global user.signingkey 112BEF7089D486F1770B8A0E9AADD83172BA05B2
  # Optional, git will find proper key for signing when passing -S
  $ git commit -S -m "signed commit message"
  ```

- Meld

  ```bash
  sudo apt install meld
  sudo cp git-diffall /usr/bin
  git config --global diff.tool meld
  git config --global alias.diffall git-diffall
  ```

- Aliases

  ```bash
  alias gst='git status -sb'
  alias glo='git log --oneline --graph --decorate --all'
  ```

### Node.js from nodesource

```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install nodejs
sudo npm config -g set prefix $HOME/.npm-global
# Fish shell config: set -gx PATH ~/.npm-global/bin $PATH
```

### Neovim

Provide v0.11.5 nvim.appimage (renamed to vim) from [nvim release page](https://github.com/neovim/neovim/releases/).

```bash
sudo cp vim /usr/bin/vim
git clone git@github.com:HuaTsai/nvim.git ~/.config/nvim
vim  # Then :MasonInstallAll, :Copilot auth
```

### [Docker](https://docs.docker.com/engine/install/ubuntu)

```bash
sudo apt install ca-certificates
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
set VERSION_CODENAME (grep VERSION_CODENAME /etc/os-release | string split "=" | tail -n1 | string trim)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $VERSION_CODENAME stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
# Reboot
docker run hello-world
```

- GPU Support - [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

  ```bash
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt update
  sudo apt install -y nvidia-container-toolkit
  ```

- Run with X11 Display

  ```bash
  xhost +local:docker
  docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu:24.04 bash

  # Test X11
  apt update
  apt install -y x11-apps
  xeyes

  # Test QT
  apt install -y qtbase5-examples
  /usr/lib/x86_64-linux-gnu/qt5/examples/widgets/widgets/analogclock/analogclock
  ```

### Python 3 with uv

- Install UV: `curl -fsSL https://astral.sh/uv/install.sh | sh`
- Manage virtual environment in project folder

  ```bash
  uv init
  uv venv
  uv add <package>  # will add dependencies in pyproject.ml
  uv pip install <package>
  ```

### AI Toolkits

- Install keyring

  ```bash
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i cuda-keyring_1.1-1_all.deb
  rm cuda-keyring_1.1-1_all.deb
  sudo apt update
  ```

  - Multiple drivers will be available after `sudo apt update`
  - Sticking on the insatlled ubuntu native driver is better

- Install relevent tools

  - Use `apt show` to check package and its dependencies before installation

  ```bash
  sudo apt install nvtop
  sudo apt install cuda      # nvidia-driver-580-open & cuda 13.0.2-1
  sudo apt install cudnn     # 9.16.0.29-1
  sudo apt install tensorrt  # 10.11.0.33-1
  ```

  - If MOK enrollment shows, set password and then reboot
    - If missed, run `sudo secureboot-policy --enroll-key` to type password again

- Environment

  ```bash
  set -gx PATH /usr/src/tensorrt/bin /usr/local/cuda/bin $PATH
  ```

- CUDA Examples

  ```bash
  sudo apt install freeglut3-dev libglfw3-dev libfreeimage-dev
  git clone -b v12.8 --depth 1 git@github.com:NVIDIA/cuda-samples.git
  cd cuda-samples
  cmake -B build
  cmake --build build -j$(nproc)
  build/Samples/1_Utilities/deviceQuery/deviceQuery
  ```

### Pytorch

- Follow guide in official [website](https://pytorch.org/get-started/locally/), using `uv add torch` is suggested

### [ROS2](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html)

| Distro              | EOL date  | Tier 1 Platform      |
| ------------------- | --------- | -------------------- |
| Rolling             | N/A       | N/A                  |
| Kilted Kaiju        | Dec. 2026 | Ubuntu 24.04 (Noble) |
| Jazzy Jalisco       | May 2029  | Ubuntu 24.04 (Noble) |
| Iron Irwini         | Dec. 2024 | Ubuntu 22.04 (Jammy) |
| Humble Hawksbill    | May 2027  | Ubuntu 22.04 (Jammy) |
| Galactic Geochelone | Dec. 2022 | Ubuntu 20.04 (Focal) |
| Foxy Fitzroy        | Jun. 2023 | Ubuntu 20.04 (Focal) |

```bash
sudo add-apt-repository universe
set ROS_APT_SOURCE_VERSION (curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
set VERSION_CODENAME (grep VERSION_CODENAME /etc/os-release | string split "=" | tail -n1 | string trim)
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/"$ROS_APT_SOURCE_VERSION"/ros2-apt-source_"$ROS_APT_SOURCE_VERSION"."$VERSION_CODENAME"_all.deb"
sudo apt install /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade
sudo apt install ros-<distro>-desktop ros-dev-tools  # or rolling
sudo rosdep init
rosdep update
```

- Important packages
  - `ros-<distro>-image-transport-plugins`
  - `ros-<distro>-vision-msgs`
  - `ros-<distro>-vision-msgs-rviz-plugins`
  - `ros-<distro>-pcl-conversions`
- Uninstall
  - `sudo apt purge '~nros-rolling*' && sudo apt autoremove`
- Install dependencies from ros2 workspace
  - `rosdep install --from-paths src --ignore-src -r`
- Shell config

  ```bash
  set -gx ROS_LOCALHOST_ONLY 1
  bass source /opt/ros/<distro>/setup.bash
  alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
  alias crm='rm -rf build install log'
  alias sis='bass source install/setup.bash'
  ```

### Sound Loopback

```bash
# Ubuntu 24.04
sudo apt install pipewire helvum

# Ubuntu 22.04
sudo apt install pavucontrol
pactl load-module module-loopback latency_msec=1
pavucontrol  # at recording tab, select "Monitor of <output device>"
```

### Yazi

Follow the [official guide](https://yazi-rs.github.io/docs/installation/)

```bash
sudo apt install ffmpeg 7zip jq poppler-utils fd-find ripgrep fzf zoxide imagemagick
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup update
cargo install --force yazi-build
```

Fish shell wrapper

```bash
function y
    set tmp (mktemp -t "yazi-cwd.XXXXXX")
    yazi $argv --cwd-file="$tmp"
    if read -z cwd < "$tmp"; and [ -n "$cwd" ]; and [ "$cwd" != "$PWD" ]
        builtin cd -- "$cwd"
    end
    rm -f -- "$tmp"
end
```

### Background Service

```bash
sudo apt install variety
```

### Obsidian

- Download appimage, rename `obsidian` and move to `/usr/bin/obsidian`
- Create desktop entry `~/.local/share/applications/obsidian.desktop`

  ```ini
  [Desktop Entry]
  Name=Obsidian
  Comment=Knowledge base and note-taking application
  Exec=/usr/bin/obsidian --no-sandbox %u
  Terminal=false
  Type=Application
  Icon=obsidian
  Categories=Office;TextEditor;
  MimeType=x-scheme-handler/obsidian;
  StartupWMClass=obsidian
  ```

- Donwload [icon](https://obsidian.md/blog/new-obsidian-icon/) named `obsidian.png` and move to `~/.local/share/icons/obsidian.png`
- Run `update-desktop-database ~/.local/share/applications/`

### Other Apps

- 1password
- astah
- discord
- surfshark
- obsidian

### Fortune and insult mode

- Edit `sudo visudo` to add `Defaults insults`
- Run `sudo apt install -y cowsay lolcat fortune`
- Fish shell add

  ```bash
  if status is-interactive
      fortune | cowsay | lolcat
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
  ```
