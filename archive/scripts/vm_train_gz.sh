#!/usr/bin/env bash
# vm_train_gz.sh —— 等采集完成后启动域内微调
cd /home/earl/vla/code/test
for i in $(seq 1 40); do
  [ -f gz_data.npz ] && break
  sleep 10
done
if [ ! -f gz_data.npz ]; then
  echo "NO-NPZ"; tail -5 collect.log; exit 1
fi
ls -la gz_data.npz
tail -3 collect.log
nohup python3 -u train_gz.py --data gz_data.npz --out runs/gz_model \
  --init pretrained_v3/ckpt.pt --epochs 12 --lr 2e-4 --bs 128 > train_gz.log 2>&1 &
echo "TRAIN-LAUNCHED pid $!"
