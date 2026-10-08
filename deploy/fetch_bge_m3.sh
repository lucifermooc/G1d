#!/bin/bash
# 从 hf-mirror 下载 / 补齐 ~/models/bge-m3（实体目录，不用 HF 缓存）。可重复运行：已完整的文件跳过，大文件续传。
set -u
D=~/models/bge-m3
B=https://hf-mirror.com/BAAI/bge-m3/resolve/5617a9f61b028005a4858fdac845db406aefb181
SHA=b5e0ce3470abf5ef3831aa1bd5553b486803e83251590ab7ff35a117cf6aad38
SIZE=2271145830
mkdir -p $D/1_Pooling
get() {  # get <相对路径>：小文件，不存在才下
  [ -s "$D/$1" ] && { echo "have $1"; return; }
  for i in 1 2 3 4 5; do curl -sfL --retry 3 -o "$D/$1.part" "$B/$1" && mv "$D/$1.part" "$D/$1" && { echo "ok $1"; return; }; sleep 5; done
  echo "FAIL $1"
}
for f in config.json config_sentence_transformers.json modules.json sentence_bert_config.json 1_Pooling/config.json \
         special_tokens_map.json tokenizer_config.json tokenizer.json sentencepiece.bpe.model colbert_linear.pt sparse_linear.pt; do
  get $f
done

# 大文件：rsync 断在一半留下的临时文件接着续传
W=$D/pytorch_model.bin
P=$D/pytorch_model.bin.part
[ -f "$W" ] && [ "$(stat -c %s "$W")" != "$SIZE" ] && rm -f "$W"   # 不完整的正式文件删掉
[ ! -f "$P" ] && ls $D/.pytorch_model.bin.* >/dev/null 2>&1 && mv "$(ls -S $D/.pytorch_model.bin.* | head -1)" "$P"
rm -f $D/.pytorch_model.bin.*
if [ ! -f "$W" ]; then
  echo "resume from $(stat -c %s "$P" 2>/dev/null || echo 0) bytes $(date +%T)"
  for i in $(seq 20); do
    curl -sL -C - --retry 5 -o "$P" "$B/pytorch_model.bin" && [ "$(stat -c %s "$P")" = "$SIZE" ] && break
    echo "retry $i at $(stat -c %s "$P") bytes $(date +%T)"; sleep 10
  done
  mv "$P" "$W"
fi
echo "sha256 check $(date +%T)"
if [ "$(sha256sum "$W" | cut -d' ' -f1)" = "$SHA" ]; then echo "MODEL_OK"; else
  echo "sha mismatch -> redownload from scratch"; rm -f "$W"
  curl -sL --retry 5 -o "$P" "$B/pytorch_model.bin" && mv "$P" "$W"
  [ "$(sha256sum "$W" | cut -d' ' -f1)" = "$SHA" ] && echo "MODEL_OK" || echo "MODEL_FAIL"
fi
ls -la $D
