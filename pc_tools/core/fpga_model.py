# -*- coding: utf-8 -*-
"""
fpga_model.py - 新缺陷检测算法的 bit-accurate 定点模型 + 权重生成
算法(与RTL一一对应):
  1) 512点采集帧内找|x|峰值位置 imp (即主冲击)
  2) 取 seg[i] = x[imp+16+i], i=0..383  (跳过16点冲击瞬态)
     若 imp+400 > 512 则该击 INVALID
  3) 自动缩放: sh = max(0, bits(peak)-15), s[i] = seg[i]>>sh  (16位有符号)
  4) 整数自相关 r[k] = sum_{i=0}^{383-k} s[i]*s[i+k], k=0..135
  5) 二次缩放: rsh = max(0, bits(r[0])-30), rs[k] = r[k]>>rsh (31位内)
  6) 打分 score_c = sum_{k=8}^{135} w_c[k]*rs[k] + b_c*rs[0]   (Q12整数权重)
  7) pred = argmax(score), margin = top1-top2 < rs0*MARGIN_Q -> 低置信
类别: 0=good 1=d200 2=d307 3=d695 4=d800
验证: A 偶训奇测(定点) + 全量训练权重导出 verilog include + 仿真测试向量
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

import numpy as np
from pathlib import Path
from core.autocorr_survey import load_hit, DATA

SKIP, SEGLEN, NLAG = 16, 384, 136
LAG0 = 8
IMP_WIN = 112              # 冲击峰搜索窗(帧前段, 触发点~64附近)
QW = 12                    # 权重定点位数
RIDGE = 1e-2
CLS = {5:0,6:0,7:0,8:0, 21:1,22:1, 13:2,14:2,15:2,16:2,
       9:3,10:3,11:3,12:3, 17:4,18:4,19:4,20:4}
NAMES = ["good","d200","d307","d695","d800"]
DIST = [0,200,307,695,800]
LIGHT = {5,7,9,11,13,15,17,19,21,22}
MIN_PEAK = 300000
MARGIN_Q = 82              # 0.02 * 4096, 低置信门限

def fx_feature(x):
    """定点特征: 返回 int64 r[0..135], 或 None(INVALID)
    冲击峰检测: 在帧前段[0,IMP_WIN)内搜索最大峰, 确保捕获的是初始锤击
    而非后续的强缺陷反射(如d800铁头, 反射可达撞击峰1.5倍)。
    RTL中同样在触发窗口内锁定冲击位置。"""
    ax = np.abs(x)
    search_end = min(IMP_WIN, len(x))
    peak = int(ax[:search_end].max())
    if peak < MIN_PEAK: return None
    imp = int(np.argmax(ax[:search_end]))
    if imp + SKIP + SEGLEN > len(x): return None
    seg = x[imp+SKIP : imp+SKIP+SEGLEN].astype(np.int64)
    sh = max(0, int(peak).bit_length() - 15)
    s = seg >> sh
    r = np.zeros(NLAG, dtype=np.int64)
    for k in range(NLAG):
        r[k] = int(np.dot(s[:SEGLEN-k], s[k:]))
    if r[0] <= 0: return None
    return r

def load_all(maxn=600):
    data = []
    for g in sorted(DATA.iterdir()):
        if not g.is_dir(): continue
        try: gi = int(g.name.split("_")[0])
        except ValueError: continue
        if gi not in CLS: continue
        files = sorted(g.glob("hit_*.csv"))
        if len(files) > maxn: files = files[::len(files)//maxn]
        for idx,p in enumerate(files):
            try: x,_ = load_hit(p)
            except Exception: continue
            if len(x) < 512: continue
            r = fx_feature(x)
            if r is None: continue
            data.append((gi, CLS[gi], gi in LIGHT, idx, r))
    return data

def train_weights(data):
    """浮点训练(v=r/r0), 输出Q12整数权重 (129 x 5): 128个lag权重 + r0偏置"""
    X = np.array([r[LAG0:]/r[0] for *_ , r in data])
    Y = np.zeros((len(X), 5))
    for i,(_,c,_,_,_) in enumerate(data): Y[i,c] = 1.0
    Xa = np.hstack([X, np.ones((len(X),1))])
    A = Xa.T @ Xa + RIDGE*len(X)*np.eye(Xa.shape[1])
    Wf = np.linalg.solve(A, Xa.T @ Y)            # (129,5)
    Wq = np.round(Wf * (1 << QW)).astype(np.int64)
    Wq = np.clip(Wq, -32768, 32767)              # 16位有符号
    return Wf, Wq

def fx_predict(Wq, r):
    """与RTL一致: rsh二次缩放后打分"""
    r0 = int(r[0])
    rsh = max(0, r0.bit_length() - 30)
    rs0 = r0 >> rsh
    scores = []
    for c in range(5):
        acc = 0
        for k in range(LAG0, NLAG):
            rk = int(r[k])
            rsk = rk >> rsh if rk >= 0 else -((-rk) >> rsh)  # 算术右移等价
            acc += int(Wq[k - LAG0, c]) * rsk
        acc += int(Wq[128, c]) * rs0
        scores.append(acc)
    order = np.argsort(scores)[::-1]
    top1, top2 = int(order[0]), int(order[1])
    margin = scores[top1] - scores[top2]         # 与 rs0*4096 同标度
    return top1, margin, rs0

def evaluate(Wq, test, tag, margin_q=None):
    n_ok=0; g_ok=0;g_t=0;b_ok=0;b_t=0; loc_ok=0;loc_t=0; rej=0
    for _,c,_,_,r in test:
        pred, margin, r0 = fx_predict(Wq, r)
        if margin_q is not None and margin < margin_q * r0:
            rej += 1
            continue
        n_ok += (pred==c)
        if c==0:
            g_t+=1; g_ok+=(pred==0)
        else:
            b_t+=1; b_ok+=(pred!=0)
            if pred!=0:
                loc_t+=1; loc_ok+=(pred==c)
    tot = g_t + b_t
    return (f"{tag}: n={tot} rej={rej} acc5={n_ok/max(tot,1):.3f} "
            f"好判好={g_ok/max(g_t,1):.3f} 坏判坏={b_ok/max(b_t,1):.3f} "
            f"定位对={loc_ok/max(loc_t,1):.3f}")

def evaluate_vote(Wq, test, tag, k=5, margin_q=None):
    bygrp={}
    for gi,c,_,idx,r in test: bygrp.setdefault(gi,[]).append((idx,c,r))
    n_ok=0;n_tot=0;g_ok=0;g_t=0;b_ok=0;b_t=0;loc_ok=0;loc_t=0
    for gi,items in bygrp.items():
        items.sort(key=lambda t:t[0])
        for i in range(0,len(items)-k+1,k):
            chunk=items[i:i+k]; c=chunk[0][1]
            votes=[0]*5; nv=0
            for _,_,r in chunk:
                p,mg,r0 = fx_predict(Wq,r)
                if margin_q is not None and mg < margin_q*r0: continue
                votes[p]+=1; nv+=1
            if nv==0: continue
            pred=int(np.argmax(votes)); n_tot+=1
            n_ok+=(pred==c)
            if c==0: g_t+=1; g_ok+=(pred==0)
            else:
                b_t+=1; b_ok+=(pred!=0)
                if pred!=0: loc_t+=1; loc_ok+=(pred==c)
    return (f"{tag} {k}击投票: n={n_tot} acc5={n_ok/max(n_tot,1):.3f} "
            f"好判好={g_ok/max(g_t,1):.3f} 坏判坏={b_ok/max(b_t,1):.3f} "
            f"定位对={loc_ok/max(loc_t,1):.3f}")

def export_verilog(Wq, path):
    """导出 defect_lda_weights.vh: 模块内 include 的权重查表函数
    lda_weight(cls, idx): idx=0..127 对应 lag k=idx+8 的权重, idx=128 为 r0 偏置"""
    lines = ["// 自动生成: python tools/fpga_model.py  (勿手改)",
             "// LDA权重 Q12, 5类 x 129 (idx=0..127 -> lag k=idx+8, idx=128 -> r0偏置)",
             "function signed [15:0] lda_weight;",
             "    input [2:0] cls;",
             "    input [7:0] idx;",
             "    begin",
             "        case ({cls, idx})"]
    for c in range(5):
        for k in range(129):
            v = int(Wq[k, c]) & 0xFFFF
            lines.append(f"            {{3'd{c}, 8'd{k}}}: lda_weight = 16'h{v:04X}; // {NAMES[c]} "
                         f"{'bias' if k==128 else 'lag'+str(k+LAG0)} = {int(Wq[k,c])}")
    lines += ["            default: lda_weight = 16'h0000;",
              "        endcase",
              "    end",
              "endfunction"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def export_mem(Wq, path):
    """导出 defect_lda_weights.mem: 供 $readmemh 初始化 M9K ROM。
    线性地址 addr = cls*256 + idx (与 RTL 中 {w_c_addr[2:0], w_idx_addr[7:0]} 一致);
    5 类 x 256 = 1280 行, 每行 16 位十六进制; idx=129..255 补 0 (未用)。"""
    lines = []
    for c in range(5):
        for a in range(256):
            v = (int(Wq[a, c]) & 0xFFFF) if a < 129 else 0
            lines.append(f"{v:04X}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def export_testvec(Wq, outdir):
    """导出仿真测试向量: 每类自动拣2个"模型判对"的样本 + 1个INVALID样本
    hit_NN.memh: 512行24位补码hex; manifest.txt: 预期结果(与RTL比对)"""
    outdir.mkdir(parents=True, exist_ok=True)
    picks = [  # (组号, 期望类, 备注)
        (5,  0, "good轻塑"), (8,  0, "good重铁"),
        (21, 1, "d200塑"),   (22, 1, "d200铁"),
        (13, 2, "d307轻塑"), (16, 2, "d307重铁"),
        (9,  3, "d695轻塑"), (12, 3, "d695重铁"),
        (17, 4, "d800轻塑"), (20, 4, "d800重铁"),
    ]
    mani = []
    n = 0

    def write_vec(x, note, exp):
        nonlocal n
        fn = f"hit_{n:02d}.memh"
        with open(outdir / fn, "w") as f:
            for v in x:
                f.write(f"{int(v) & 0xFFFFFF:06X}\n")
        mani.append(f"{fn} {note} expect: {exp}")
        n += 1

    for gi, cls, note in picks:
        gdirs = [g for g in sorted(DATA.iterdir())
                 if g.is_dir() and g.name.startswith(f"{gi:02d}_")]
        if not gdirs: continue
        files = sorted(gdirs[0].glob("hit_*.csv"))
        found = 0
        for p in files:
            try: x, _ = load_hit(p)
            except Exception: continue
            if len(x) < 512: continue
            x = x[:512].astype(np.int64)
            r = fx_feature(x)
            if r is None: continue
            pred, margin, rs0 = fx_predict(Wq, r)
            if pred != cls: continue
            lowc = 1 if margin < MARGIN_Q * rs0 else 0
            if lowc: continue                      # 只取高置信样本便于比对
            exp = f"{'NORMAL' if pred==0 else 'DEFECT'} cls={pred} dist={DIST[pred]} lowconf=0"
            write_vec(x, f"grp={gi} {note}", exp)
            found += 1
            if found >= 1: break

    # INVALID 样本1: 峰值过小 (02组是256点旧格式, 改用合成小幅噪声帧)
    rng = np.random.default_rng(7)
    x = rng.integers(-80000, 80000, size=512, dtype=np.int64)  # 峰值<MIN_PEAK
    write_vec(x, "合成噪声(峰值过小)", "INVALID")
    # INVALID 样本2: 二次弹跳(imp>=IMP_WIN), 从铁头组里找
    for gi in (20, 19, 12):
        gdirs = [g for g in sorted(DATA.iterdir())
                 if g.is_dir() and g.name.startswith(f"{gi:02d}_")]
        if not gdirs: continue
        done = False
        for p in sorted(gdirs[0].glob("hit_*.csv"))[:80]:
            try: x, _ = load_hit(p)
            except Exception: continue
            if len(x) < 512: continue
            x = x[:512].astype(np.int64)
            ax = np.abs(x)
            if int(ax.max()) >= MIN_PEAK and int(np.argmax(ax)) >= IMP_WIN:
                write_vec(x, f"grp={gi} 二次弹跳(imp>={IMP_WIN})", "INVALID")
                done = True
                break
        if done: break
    with open(outdir / "manifest.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(mani) + "\n")
    return mani

def main():
    print("loading...")
    data = load_all()
    print("total:", len(data))
    out=[]
    # A 偶训奇测(定点)
    tr=[d for d in data if d[3]%2==0]; te=[d for d in data if d[3]%2==1]
    _, Wq = train_weights(tr)
    out.append(evaluate(Wq, te, "A 偶训奇测/定点"))
    out.append(evaluate(Wq, te, "A +margin拒判(0.02)", margin_q=MARGIN_Q))
    out.append(evaluate_vote(Wq, te, "A", 5))
    out.append(evaluate_vote(Wq, te, "A", 3))
    # 跨力度
    tr=[d for d in data if d[2]]; te=[d for d in data if not d[2]]
    _, Wq2 = train_weights(tr)
    out.append(evaluate(Wq2, te, "D 轻敲->重敲/定点"))
    out.append(evaluate_vote(Wq2, te, "D", 5))
    # 全量训练 -> 部署权重
    Wf, Wq_full = train_weights(data)
    out.append(evaluate(Wq_full, data, "自检(全量训练全量测,仅供参考)"))
    rtl = Path(__file__).parent.parent / "rtl" / "defect_lda_weights.vh"
    export_verilog(Wq_full, rtl)
    mem = Path(__file__).parent.parent / "rtl" / "defect_lda_weights.mem"
    export_mem(Wq_full, mem)
    np.save(Path(__file__).parent.parent/"tmp"/"lda_Wq_full.npy", Wq_full)
    out.append(f"权重已导出: {rtl}")
    out.append(f"ROM 初始化文件: {mem}")
    mani = export_testvec(Wq_full, Path(__file__).parent.parent / "sim" / "lda_vectors")
    out.append("测试向量:")
    out.extend("  " + m for m in mani)
    txt="\n".join(out); print(txt)
    with open(Path(__file__).parent.parent/"tmp"/"fpga_model_eval.txt","w",encoding="utf-8") as f:
        f.write(txt)

if __name__=="__main__":
    main()
