# Ubuntu-Setup

## Installation

1. Download the Ubuntu ISO image
   - [Focal Fossa 20.04](https://releases.ubuntu.com/20.04)
   - [Jammy Jellyfish 22.04](https://releases.ubuntu.com/22.04)
   - [Nobel Numbat 24.04](https://releases.ubuntu.com/24.04)
2. Create a bootable USB flash drive via Ubuntu suggested tool [balenaEtcher](https://etcher.balena.io/)
3. Follow the steps to install

### Recover the USB Drive

- After creating the drive, it may be no longer accessed by windows normally.
- To recover the drive, use following commands

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

## Post-Installation Setup

### Time Synchronization

For dual system, we can to synchronize their time via `sudo timedatectl set-local-rtc 1`

### Personal Setups

- Appearance
  - Auto-hide the dock
  - Display resolutions
- Dock apps
  - Browser
  - Terminal
    - Check custom font
    - Font CaskaydiaCove Nerd Font Mono
      - [Nerd font](https://www.nerdfonts.com/font-downloads)
      - Install CaskaydiaCove Nerd Font `CaskaydiaCoveNerdFontMono-Regular.ttf`
      - Logout and then login
    - Size 22
    - TokyoNight
      - Foreground color: #a9b1d6
      - Background color: #1a1b26
  - Visual Studio Code
    - Visit [official website](https://code.visualstudio.com/) and install via `dpkg`
  - Files
  - Software & Updates
- Language
  - Settings > Region & Language > Manage Installed Languages
  - Install / Remove Languages > Chinese (traditional)
  - Logout and then login
  - Keyboard > Input Source > Chinese (Taiwan) > Add Chinese (Chewing)

### Basic Tools

```bash
sudo apt update && sudo apt upgrade
sudo apt install aptitude btop build-essential curl dos2unix git htop hyperfine libfuse2 locate neofetch net-tools openssh-server python3-venv software-properties-common tldr tree
sudo ln -s /usr/bin/python3 /usr/bin/python
```

### Multiple `g++` Versions (Optional)

For upstream versions, add software sources [ToolChain](https://wiki.ubuntu.com/ToolChain) via ppa.

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
```

Assume we want to select choose `g++` from `g++-11` and `g++-14`.

```bash
# <path/to/symlink> <command_name> <path/to/command_binary> <priority>
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 90
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 90
```

Default g++ version will be `g++-14` since it has higher priority.  
To configure g++ version, run `sudo update-alternatives --config g++`

### SSH

```bash
$ ssh-keygen
Generating public/private rsa key pair.
Enter file in which to save the key (/home/hua/.ssh/id_rsa):
Enter passphrase (empty for no passphrase):
Enter same passphrase again:
Your identification has been saved in /home/hua/.ssh/id_rsa
Your public key has been saved in /home/hua/.ssh/id_rsa.pub
The key fingerprint is:
SHA256:9fyJA503gmI1gyI1qjvcrRD2WBN8L1e7bYnAMiXrJXc hua@hua
The key's randomart image is:
+---[RSA 3072]----+
|      o          |
|   . o . .       |
|    = + o *      |
|   . + B + O .   |
|  + o * S E * o  |
| o B + X + = * o |
|  * o o   o * o  |
|   o .     . .   |
|    .            |
+----[SHA256]-----+
$ cat ~/.ssh/id_rsa.pub  # SSH public key
$ ssh-copy-id user@remote-ip  # type password
$ cat ~/.ssh/config
Host nickname
  User user
  Hostname remote-ip
$ ssh nickname
```

### Git

- Git

  ```bash
  git config --global user.name "HuaTsai"
  git config --global user.email "huatsai42@gmail.com"
  ```

- SSH key and GPG Key

  ```bash
  $ cat ~/.ssh/id_rsa.pub  # Copy the output public key to github settings
  $ gpg --list-secret-keys --keyid-format LONG  # Check for existing GPG keys
  $ gpg --full-generate-key  # Generate a new one, if we don't have a GPG key
  # Follow the prompts and make sure to use the same email address that we use for GitHub.
  $ gpg --list-secret-keys --keyid-format LONG
  gpg: checking the trustdb
  gpg: marginals needed: 3  completes needed: 1  trust model: pgp
  gpg: depth: 0  valid:   1  signed:   0  trust: 0-, 0q, 0n, 0m, 0f, 1u
  /home/hua/.gnupg/pubring.kbx
  ----------------------------
  sec   rsa3072/032515EA599EED90 2024-04-21 [SC]
          DC2F35213D386833573A08E7032515EA599EED90
  uid                 [ultimate] HuaTsai <huatsai42@gmail.com>
  ssb   rsa3072/C5ABE49697032BD9 2024-04-21 [E]
  $ gpg --armor --export 032515EA599EED90
  # Copy the GPG key, beginning with `-----BEGIN PGP PUBLIC KEY BLOCK-----` and ending with `-----END PGP PUBLIC KEY BLOCK-----` to github settings
  $ git config --global user.signingkey 032515EA599EED90
  $ git commit -S -m "commit message"  # pass -S to sign
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

### Fish Shell

```bash
sudo apt-add-repository ppa:fish-shell/release-3
sudo apt update
sudo apt install fish
chsh -s $(which fish)
curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher
echo "set -gx fish_greeting" >> ~/.config/fish/config.fish
# Logout and then login
```

### Node.js (from nodesource)

```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install nodejs
```

### Neovim

Provide v0.10.4 nvim.appimage (renamed to vim) from [nvim release page](https://github.com/neovim/neovim/releases/).

```bash
sudo mv vim /usr/bin/vim
git clone git@github.com:HuaTsai/nvim.git ~/.config/nvim
vim
```

### Jump Command: `z`

- Bash

  ```bash
  git clone git@github.com:rupa/z.git
  echo ". ~/z/z.sh" >> ~/.bashrc
  ```

- Fish

  ```bash
  fisher install jethrokuan/z
  ```

### [Docker](https://docs.docker.com/engine/install/ubuntu)

```bash
sudo apt install ca-certificates
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
docker run hello-world
```

- GPU Support - [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

  ```bash
  # In bash shell
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt update
  sudo apt install -y nvidia-container-toolkit
  ```

- Run with X11 Display

  ```bash
  xhost +local:docker
  docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu:22.04 bash

  # Test X11
  apt install -y x11-apps
  xeyes

  # Test QT
  apt install -y qtbase5-examples
  /usr/lib/x86_64-linux-gnu/qt5/examples/widgets/widgets/analogclock/analogclock
  ```

### AI Toolkits

- CUDA 11.8 + cuDNN 8.9.7 + TensorRT 8.6.1 (Example on Ubuntu 22.04)

  ```bash
  # CUDA 11.8
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i cuda-keyring_1.1-1_all.deb
  sudo apt update
  sudo apt install cuda=11.8.0-1

  # cudnn 8.9.7
  sudo aptitude install libcudnn8-dev=8.9.7.29-1+cuda11.8
  sudo apt install libcudnn8-samples=8.9.7.29-1+cuda11.8

  # TensorRT 8.6.1.6
  sudo aptitude install tensorrt=8.6.1.6-1+cuda11.8

  # Mark packages to prevent apt upgrade (but apt install still works, prevent this call)
  sudo apt-mark hold cuda libcudnn8-dev libcudnn8-samples tensorrt
  apt-mark showhold

  # Environment (bash and fish)
  echo "PATH=/usr/src/tensorrt/bin:/usr/local/cuda/bin:$PATH" >> ~/.bashrc
  echo "set -gx PATH /usr/src/tensorrt/bin /usr/local/cuda/bin $PATH" >> ~/.config/fish/config.fish
  ```

- To compile CUDA examples, run `sudo apt insteall freeglut3-dev libglfw3-dev libfreeimage-dev`
- If you miss the MOK enrollment, run `sudo secureboot-policy --enroll-key` to type password again

### Anaconda

- Download and install [Anaconda](https://www.anaconda.com/download)
- Install to location `~/anaconda3`
- Bash

  ```bash
  ~/anaconda3/bin/conda init
  conda config --set auto_activate_base false
  ```

- Fish

  ```bash
  ~/anaconda3/bin/conda init fish
  conda config --set auto_activate_base false
  ```

### ROS2

```bash
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# In bash shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade
sudo apt install ros-<distro>-desktop ros-dev-tools
sudo rosdep init
rosdep update
```

To install dependencies, run `rosdep install --from-paths src --ignore-src -r` in ros2 worksapce.

- Environment

  - Bash config in `~/.bashrc`

    ```bash
    source /opt/ros/<distro>/setup.bash
    alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
    alias crm='rm -rf build install log'
    ```

  - Fish config in `~/.config/fish/config.fish` (need `fisher install edc/bass`)

    ```bash
    bass source /opt/ros/<distro>/setup.bash
    alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
    alias crm='rm -rf build install log'
    alias sis='source install/setup.bash'
    ```

- Some important packages
  - `ros-$ROS_DISTRO-image-transport-plugins`

### Pytorch

- Follow guide in official [website](https://pytorch.org/get-started/locally/)

### Install SSD

```bash
sudo fdisk -l  # Check disk, it should appear without partition
sudo fdisk /dev/nvme1n1  # Create partitions, use default values and finally write it
sudo mkfs -t ext4 -c /dev/nvme1n1p1  # Then it should appears
```

### Access SSD

If SSD is not shown in the dock, we can run the following commands

```bash
$ sudo fdisk -l  # Check target disk
$ sudo blkid /dev/nvme1n1p1  # Show disk UUID
$ sudo mkdir /media/SSD && sudo chown user:user /media/SSD  # Create directory and change owner
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

  ```bash
  function grn
      grep -rn --color=auto --exclude-dir={.git,node_modules,__pycache__} $argv
  end
  ```

### Fortune and insult mode

- Edit `sudo visudo` to add `Defaults insults`
- Run `sudo apt install cowsay lolcat fortune`
- Fish shell add

   ```bash
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
   
   fortune | cowsay | lolcat
   ```
