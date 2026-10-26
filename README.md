# MAPF

Multi-Agent Path Finding (MAPF) の学習・実験用プロジェクト。標準ライブラリのみで動作します(テストに pytest)。

## 実装済み

| 手法 | 完全性 | 最適性 | 場所 |
|---|---|---|---|
| Prioritized Planning | なし | なし | `src/mapf/prioritized.py` |
| Conflict-Based Search (CBS) | あり | sum-of-costs 最適 | `src/mapf/cbs.py` |

- 低レベル探索: 時空間 A*(待機あり、頂点・辺制約、ゴール到達後の占有を考慮) — `astar.py`
- 衝突検出・解の検証(頂点衝突 / 交換衝突) — `validate.py`
- 可視化: ASCII、アニメーション SVG — `visualize.py`

## 使い方

```bash
pip install -e '.[dev]'
python -m mapf maps/corridor.txt -a cbs --ascii
python -m mapf maps/crossing.txt -a pp --svg out.svg
python -m mapf --random maps/room.txt -n 8 --seed 3
pytest
```

## シナリオ形式

マップ(`.` 空き / `#` 障害物)、`---`、各エージェントの `start_row start_col goal_row goal_col`。`//` 以降はコメント。座標は `(row, col)`、左上が原点。

## 前提

4近傍+待機、全エージェント同時移動、頂点衝突と交換衝突を禁止。ゴール到達後もそのセルに留まり続ける。

## 今後の候補

PIBT / EECBS などの追加、MovingAI ベンチマーク読み込み、実験スクリプト、Lifelong MAPF。
