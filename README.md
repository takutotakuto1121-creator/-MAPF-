# MAPF

Multi-Agent Path Finding (MAPF) の学習・実験用プロジェクト。実行時の依存は **標準ライブラリのみ**(開発用に pytest / ruff)。

![demo](docs/demo.svg)

## 実装済みアルゴリズム

| 手法 | 完全性 | 最適性 | 特徴 | 場所 |
|---|---|---|---|---|
| Prioritized Planning (PP) | なし | なし | 速い。優先順位が悪いと失敗する | `src/mapf/prioritized.py` |
| Conflict-Based Search (CBS) | あり | sum-of-costs 最適 | 小〜中規模向け。エージェント数に弱い | `src/mapf/cbs.py` |
| PIBT | 一般にはなし | なし | 1ステップずつ決める。大規模でも高速 | `src/mapf/pibt.py` |

共通部品: 時空間 A*(待機・頂点制約・辺制約・ゴール到達後の占有を考慮、`astar.py`)、衝突検出と解の検証(`validate.py`)、
MovingAI ベンチマーク形式の読み込み(`movingai.py`)、ASCII / アニメーション SVG 出力(`visualize.py`)。

## クイックスタート

```bash
make install                     # pip install -e '.[dev]'
make test                        # テスト
make demo ALGO=pibt AGENTS=10    # ランダムな問題を解いて ASCII で表示
make svg  ALGO=cbs  SEED=3       # out.svg にアニメーションを出力
make bench                       # cbs / pp / pibt を比較
make help                        # ターゲット一覧
```

CLI を直接使う場合:

```bash
python -m mapf maps/corridor.txt -a cbs --ascii
python -m mapf --random maps/room.txt -n 8 --seed 3 -a pp --svg out.svg
python -m mapf --movingai empty-8-8.map empty-8-8-random-1.scen -n 20 -a pibt
python -m mapf.bench --map maps/room.txt -n 4 8 12 --trials 20 --timeout 2
```

終了コード: `0` 成功 / `1` 解なし(または探索上限) / `2` 解が検証に失敗。

## 入力形式

**独自形式**(`maps/*.txt`): マップ(`.` 空き / `#` 障害物)、`---`、各エージェントの `start_row start_col goal_row goal_col`。`//` 以降はコメント。座標は `(row, col)`、左上が原点。

**MovingAI 形式**: [MAPF ベンチマーク](https://movingai.com/benchmarks/mapf/) の `.map` / `.scen` をそのまま読めます(`.scen` の座標は x=列, y=行)。`.` `G` `S` を通行可、それ以外を障害物として扱います。

## 問題設定

4近傍+待機、全エージェント同時移動。頂点衝突と交換(スワップ)衝突を禁止。ゴール到達後もそのセルに留まり、他のエージェントを塞ぎ続けます。目的関数は sum-of-costs(各エージェントが最終的にゴールに着く時刻の合計)と makespan。

## ベンチマーク例

`maps/room.txt`(8x8)、各20問、CBS のタイムアウト2秒。コスト・時間は全手法が解けた問題の平均。`solved` は各手法が解けた数。

| agents | algo | solved | avg SoC | avg makespan | avg time [s] |
|---:|---|---:|---:|---:|---:|
| 4 | cbs | 20/20 | 22.8 | 8.1 | 0.0012 |
| 4 | pp | 20/20 | 23.6 | 8.1 | 0.0007 |
| 4 | pibt | 18/20 | 29.8 | 10.8 | 0.0009 |
| 8 | cbs | 18/20 | 49.3 | 11.1 | 0.2910 |
| 8 | pp | 20/20 | 57.0 | 12.0 | 0.0016 |
| 8 | pibt | 17/20 | 71.4 | 15.1 | 0.0016 |
| 12 | cbs | 3/20 | 87.0 | 11.0 | 0.2654 |
| 12 | pp | 13/20 | 98.0 | 11.0 | 0.0024 |
| 12 | pibt | 10/20 | 99.0 | 12.0 | 0.0020 |

読み取れること: CBS は最もコストが小さい一方、12台ではタイムアウトが増える。PP は速くコストも近いが、混むと失敗する。
PIBT は行き止まりのある狭い地図では完走できないことがある(完全性は袋小路のない地図が前提)。12台の行は、CBS が解けた3問だけの平均で、他の行より母数が小さい点に注意。

## 開発

```bash
make check    # ruff + pytest(CI と同じ)
make fmt      # 自動整形
make cov      # カバレッジ
```

GitHub Actions(`.github/workflows/ci.yml`)で Python 3.10 / 3.12 / 3.13 の lint とテストを実行します。

## 今後の候補

ICBS / EECBS(準最適 CBS)、衝突の cardinal 分類による CBS 高速化、Lifelong MAPF、実行遅延に強い計画(Robust MAPF)、8近傍移動。
