# Ubuntu-Setup

## Installation

1. Download the Ubuntu [22.04](https://releases.ubuntu.com/22.04/ubuntu-22.04.4-desktop-amd64.iso)/[24.04](https://releases.ubuntu.com/24.04/ubuntu-24.04-desktop-amd64.iso) ISO image
2. Create a bootable USB flash drive via Ubuntu suggested tool [balenaEtcher](https://etcher.balena.io/)
3. Follow the steps to install

### Recover the USB Drive

- After creating the drive, it may be no longer accessed by windows normally.
- To recover the drive, use following commands

    ``` powershell
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

### Update and Upgrade

``` bash
sudo apt update
sudo apt upgrade
```

### Basic tools

``` bash
sudo apt install git build-essential curl python3-venv net-tools dos2unix tree htop btop neofetch hyperfine locate libfuse2
sudo ln -s /usr/bin/python3 /usr/bin/python
```

### Git

- Git

  ``` bash
  git config --global user.name "HuaTsai"
  git config --global user.email "huatsai.eed07g@nctu.edu.tw"
  ```

- SSH

  ``` bash
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
  $ cat ~/.ssh/id_rsa.pub  # Copy the output public key to github settings
  ```

  - Remote setting
  
    ``` bash
    $ ssh-copy-id user@remote-ip
    # type password
    $ cat ~/.ssh/config
    Host nickname
      User user
      Hostname remote-ip
    $ ssh nickname
    ```

- GPG Key

  ```bash
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
  uid                 [ultimate] HuaTsai <huatsai.eed07g@nctu.edu.tw>
  ssb   rsa3072/C5ABE49697032BD9 2024-04-21 [E]
  $ gpg --armor --export 032515EA599EED90
  # Copy the GPG key, beginning with `-----BEGIN PGP PUBLIC KEY BLOCK-----` and ending with `-----END PGP PUBLIC KEY BLOCK-----` to the Github account
  $ git config --global user.signingkey 032515EA599EED90
  $ git commit -S -m "commit message"
  ```

- Meld

  ``` bash
  sudo apt install meld
  sudo cp git-diffall /usr/bin
  git config --global diff.tool meld
  git config --global alias.diffall git-diffall
  ```

### NodeJS (by nodesource)

``` bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install nodejs
```

### Neovim

- Provide v0.10.0 nvim.appimage from [nvim release page](https://github.com/neovim/neovim/releases/)

``` bash
sudo apt install ripgrep libstdc++-12-dev
sudo ln -s ~/nvim.appimage /usr/bin/vim
git clone git@github.com:HuaTsai/NvChad.git ~/.config/nvim
vim  # automatically MasonInstallAll
```

### Fish Shell

``` bash
sudo apt-add-repository ppa:fish-shell/release-3
sudo apt update
sudo apt install fish
chsh -s $(which fish)
curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher
```

Logout and then login

``` bash
$ cat ~/.config/fish/config.fish
...
set -g fish_greeting
```

### z

- Bash

  ``` bash
  $ git clone git@github.com:rupa/z.git
  $ cat ~/.bashrc
  . ~/z/z.sh
  ```

- Fish

  ``` bash
  fisher install jethrokuan/z
  ```

### [Docker](https://docs.docker.com/engine/install/ubuntu)

``` bash
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo docker run hello-world
```

* Run with X11 Display

``` bash
xhost +local:docker
sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04 bash
```


### [CUDA](https://developer.nvidia.com/cuda-toolkit-archive), [cuDNN](https://developer.nvidia.com/cudnn-archive), and [TensorRT](https://developer.nvidia.com/tensorrt/download)

- Prerequisit
  - Check recommended driver: `ubuntu-drivers devices`
  - `sudo apt install nvidia-driver-535`
  - `nvidia-smi`
  - Remove old kernels `sudo apt --purge remove '*6.5.0-18*'`
- CUDA 11.6
  - Download the `cuda_11.6.0_510.39.01_linux.run` file
  - So that we can choose not to install the Nvidia 510 driver
  - `sudo sh cuda_11.6.0_510.39.01_linux.run`
- cuDNN 8.4.1
  - Download the `cudnn-local-repo-ubuntu2004-8.4.1.50_1.00-1_amd64.deb` file
  - While installation, it will show the cuda version. It is better to match this version with cuda 11.6.

  ```bash
  sudo dpkg -i cudnn-local-repo-ubuntu2004-8.4.1.50_1.00-1_amd64.deb
  # It will show a command, copy it!
  sudo cp /var/cudnn-local-repo-ubuntu2004-8.4.1.50/cudnn-local-E3EC4A60-keyring.gpg /usr/share/keyrings/
  sudo apt update
  sudo apt install libcudnn8-dev  # version 8.4.1.50-1+cuda11.6
  ```

- TensorRT 8.4.3
  - Since we install cuda via run file, so `.deb` file will not work for installing TensorRT
  - Download the `TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz` file
  - Follow the instructions Nvidia provides

  ``` bash
  tar -xzvf TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz
  cd TensorRT-8.4.3.1
  sudo apt install python3-pip
  pip install python/tensorrt-8.4.3.1-cp38-none-linux_x86_64.whl
  pip install onnx_graphsurgeon/onnx_graphsurgeon-0.3.12-py2.py3-none-any.whl
  # Verify MNIST in samples folder
  ```

- Final `.bashrc` setting

  - bash

    ``` bash
    PATH=/usr/local/cuda/bin:/home/hua/TensorRT-8.4.3.1/bin:/home/hua/.local/bin:$PATH
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:/home/hua/TensorRT-8.4.3.1/lib:$LD_LIBRARY_PATH
    ```

  - fish

    ``` bash
    set -gx PATH /usr/local/cuda-11.8/bin /home/hua/TensorRT-8.4.3.1/bin $PATH
    set -gx LD_LIBRARY_PATH /usr/local/cuda-11.8/lib64 /home/hua/TensorRT-8.4.3.1/lib $LD_LIBRARY_PATH
    ```

- Ubuntu 22.04
  - CUDA 11.8: cuda_11.8.0_520.61.05_linux.run
  - cuDNN 8.9.7(.29-1+cuda11.8): cudnn-local-repo-ubuntu2204-8.9.7.29_1.0-1_amd64.deb
  - TensorRT 8.6 GA: TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz

### Anaconda

- Download and install [Anaconda](https://www.anaconda.com/download)
- Install to location `/home/hua/anaconda3`

``` bash
Do you wish to update your shell profile to automatically initialize conda?
This will activate conda on startup and change the command prompt when activated.
If you'd prefer that conda's base environment not be activated on startup,
   run the following command when conda is activated:

conda config --set auto_activate_base false

You can undo this by running `conda init --reverse $SHELL`? [yes|no]
[no] >>> yes
conda init
conda config --set auto_activate_base false
# Check ~/.bashrc for the setting
conda create --name <env_name> python=3.11
conda activate ...
conda deactivate ...
conda env remove ...
```

For fish shell

``` bash
/home/foxconn/anaconda3/bin/conda init fish
conda config --set auto_activate_base false
# Check ~/.config/fish/config.fish for the setting
conda create --name <env_name> python=3.11
conda activate <env_name>
```

### ROS2 Humble

``` bash
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# the following command should be run in bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools
# Put "source /opt/ros/humble/setup.bash" in ~/.bashrc
sudo rosdep init
rosdep update
# Future install dependencies: rosdep install --from-paths src --ignore-src -r
```

For fish shell

``` bash
$ fisher install edc/bass
$ cat ~/.config/fish/config.fish
set -g fish_greeting
set -gx PATH /usr/local/cuda-11.8/bin /home/hua/TensorRT-8.4.3.1/bin $PATH
set -gx LD_LIBRARY_PATH /usr/local/cuda-11.8/lib64 /home/hua/TensorRT-8.4.3.1/lib $LD_LIBRARY_PATH
bass source /opt/ros/humble/setup.bash
```

- Some important packages
  - `ros-humble-image-transport-plugins`

### Pytorch

- Follow guide in official [website](https://pytorch.org/get-started/locally/)
  - `conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia`
  - pytorch: 2.3.0
  - torchvision: 0.18.0
  - torchaudio: 2.3.0

### Install SSD

``` bash
sudo fdisk -l  # Check disk, it should appear without partition
sudo fdisk /dev/nvme1n1  # Create partitions, use default values and finally write it
sudo mkfs -t ext4 -c /dev/nvme1n1p1  # Then it should appears
```

### Access SSD

If SSD is not shown in the dock, we can run the following commands

``` bash
$ sudo fdisk -l  # Check target disk
$ sudo blkid /dev/nvme1n1p1  # Show disk UUID
$ sudo mkdir /media/SSD && sudo chown user:user /media/SSD  # Create directory and change owner
$ cat /etc/fstab  # Add mount directory
...
UUID=... /media/SSD ext4 defaults 0 0
...
```
