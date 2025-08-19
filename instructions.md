## ULTRA setup and usage (Termux/ARM)

### 1) Install Git

```bash
pkg install git
```

When prompted, type `y` and press Enter to approve the installation.

### 2) Clone the repository

```bash
git clone --single-branch --branch armULTRA https://github.com/PostdataAI/ULTRA.git
```

Optionally change into the cloned repository before continuing:

```bash
cd ULTRA
```

### 3) Create and enter the build directory

```bash
mkdir -p cmake-build-release
cd cmake-build-release
```

### 4) Install CMake

```bash
pkg install cmake
```

When prompted, type `y` and press Enter to approve the installation.

### 5) Build the project

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target All --config Release
```

### 6) Download London and Swish networks (from inside `cmake-build-release`)

```bash
pkg update && pkg upgrade -y
pkg install python -y
pip install gdown
```

Now download the networks (this may take time depending on your internet speed):

```bash
gdown --folder "https://drive.google.com/drive/folders/10TCEZyB33R5RJsZrW4aWNfR1TlAd3O-X?usp=sharing"
gdown --folder "https://drive.google.com/drive/folders/1UOGrM0wC1RFD-g6zMntqcyQgOAxA-RC1?usp=sharing"
```

### 7) Verify downloads

```bash
ls
```

Ensure the downloaded network folders (e.g., `London`, `Swish`) are present.

### 8) Run the executable

```bash
./ULTRA
```

### 9) Choose and run an algorithm

For example, run `CheckRAPTORPruning`.

### 10) Provide the network path

Examples:

```bash
./London/raptor.binary
```

or

```bash
./Swish/raptor.binary
```

### 11) Enter query count

Provide the desired query count when prompted.

### 12) Run

Follow the prompts to execute the algorithm.

### 13) End

Exit when finished.


