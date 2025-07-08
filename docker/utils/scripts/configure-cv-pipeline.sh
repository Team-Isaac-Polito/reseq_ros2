#/bin/bash

pip install --upgrade-strategy only-if-needed "numpy>=1.23.0" "matplotlib>=3.3.0" "pillow>=7.1.2" "pyyaml>=5.3.1" "requests>=2.23.0" "scipy>=1.4.1" "tqdm>=4.64.0" "psutil" "py-cpuinfo" "pandas>=1.1.4" "scikit-learn==1.3.2"
pip install pyzbar "uff==0.6.9"
pip install --no-deps ultralytics "ultralytics-thop>=2.0.0"
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.2.0-cp38-cp38-linux_aarch64.whl
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.17.2+c1d70fe-cp38-cp38-linux_aarch64.whl
apt update && apt install -y libopenmpi3 libopenblas-dev libzbar0
echo "ISAAC Computer Vision Pipeline configured successfully"