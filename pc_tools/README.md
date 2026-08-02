# PC Tools — 基桩动测仪 PC 端工具集

```
pc_tools/
├── README.md
├── core/                          # 核心算法库（被其他工具导入）
│   ├── fpga_model.py              #   LDA 分类器 bit-accurate 定点模型 + 权重训练 + 导出
│   ├── autocorr_survey.py         #   全量数据加载器（load_hit, DATA）— 中心依赖
│   └── replay_analyzer.py         #   RTL defect_analyzer.v 按位精确 Python 复现
├── classify/                      # 分类判别算法
│   ├── eval_lda.py                #   LDA 线性分类器评估（岭回归 one-hot）
│   ├── eval_lda_sweep.py          #   LDA 参数扫描（窗口起点/长度/预白化）
│   ├── template_classify.py       #   归一化自相关 + 最近质心模板匹配
│   ├── eval_periodnorm.py         #   周期归一化自相关指纹分类
│   ├── discriminate_rods.py       #   305mm vs 695mm 单端区分
│   ├── end_discriminate.py        #   两端区分特征提取
│   ├── verify_defect.py           #   缺陷判据验证（比例测距法）
│   └── residual_energy.py         #   模板抵消 + 残差能量法
├── analyze/                       # 波形分析与特征提取
│   ├── analyze_v3.py              #   综合分析（LDA 训练 + 混淆矩阵 + 特征统计）
│   ├── analyze_capture.py         #   采集组汇总统计
│   ├── analyze_v2.py              #   逐帧精细分析
│   ├── analyze_waveform.py        #   波形级复核（包络 + 找峰）
│   ├── spectrum_survey.py         #   平均功率谱 + 自相关曲线对比
│   ├── ac_detail.py               #   关键 lag 自相关细节 + 次级峰结构
│   ├── separability.py            #   组间可分性统计（基于 feature_table.csv）
│   ├── feature_table.py           #   逐击特征提取 → CSV
│   ├── valley_check.py            #   波包谷底检测（RTL 波包分裂判据）
│   ├── diag_group.py              #   单组波包细节诊断
│   ├── template_stats.py          #   模板统计（对齐 + 中位包络）
│   ├── evaluate_continuous_locator.py  #   连续反射定位评估
│   └── analyze_official_trend.py       #   官方五点单调趋势验证
├── capture/                       # 串口通信与数据采集
│   ├── start_capture.py           #   采集启动器（GUI/CLI）
│   ├── uart_frame_parser.py       #   UART 二进制帧解析器
│   ├── uart_waveform_viewer.py    #   串口波形实时查看器（matplotlib）
│   └── test_frame_parser.py       #   帧解析器自测
├── app/                           # 主应用程序
│   └── pile_check.py              #   PC 端判别主程序（批量/单文件/串口监听）
└── legacy/                        # 旧版参考
    ├── analyze_pile.py            #   早期分析脚本
    └── pile_detector.py           #   早期判别脚本
```

## 快速开始

```bash
cd D:\FPGA\dian_sai\Project\God3.11

# 主程序 — 双击运行或命令行
python pc_tools/app/pile_check.py                    # 交互菜单
python pc_tools/app/pile_check.py --test --limit 30  # 批量测试
python pc_tools/app/pile_check.py --serial COM3       # 串口监听

# 深度分析
python pc_tools/analyze/analyze_v3.py

# LDA 评估
python pc_tools/classify/eval_lda.py

# 串口波形查看
python pc_tools/capture/uart_waveform_viewer.py
```

## 导入规范

所有工具从 `pc_tools/` 为根目录导入：
```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # pc_tools/

from core.fpga_model import fx_feature, fx_predict
from core.autocorr_survey import load_hit, DATA
from capture.uart_frame_parser import try_parse
```
