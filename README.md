# Simple Monte Carlo Localization

基本的な Monte Carlo Localization (MCL) の実装です．
ランドマークまでの距離に基づく観測モデルと，
差動駆動型ロボットを想定したオドメトリ運動モデルを用い，
パーティクルフィルタによりロボットの姿勢を推定します．

単純な旋回運動を与えたシミュレーションにより推定挙動を確認でき，
各時間ステップにおける推定姿勢などを CSV に記録します．

![demo](./assets/demo.gif)

+ 青: パーティクル
+ 緑: 姿勢の真値
+ 赤: 姿勢の推定値
+ 黒: ランドマーク (L1, L2, L3)


## Tested environment

+ OS: Ubuntu 22.04 LTS
+ Compiler: g++ 11.4.0
+ CMake: 3.22.1
+ OpenCV: 4.5.4


## Install & Build & Run

```bash
git clone https://github.com/nacky823/simple_mcl.git
cd simple_mcl && make
```
```bash
./build/simple_mcl
```

## Algorithm description

ロボットの状態（姿勢）を

$$
x_t = (x_t, y_t, \theta_t)
$$

制御入力を

$$
a_{t-1} = (\mathrm{rot1}, \mathrm{trans}, \mathrm{rot2})
$$

観測（ランドマークとの距離）を

$$
o_t = \\{ r_{t,k} \\}_{k=1}^{K}
$$

とし，次の確率分布をパーティクルで近似する．

$$
\mathrm{Bel}(x_t)\approx \\{(x_t^{(i)}, w_t^{(i)})\\}_{i=1}^{N}
$$

ランドマーク $\ell_k = (\ell_{k,x},\ell_{k,y})$ が既知のとき，姿勢 $x_t$ からの距離は

$$
\hat r_k(x_t)=\sqrt{(x_t-\ell_{k,x})^2 + (y_t-\ell_{k,y})^2}
$$

また，ランドマークごとの観測が条件付き独立であると仮定し，次のように表す．

$$
p(o_t \mid x_t)=\prod_{k=1}^{K} p(r_{t,k} \mid x_t)
$$

これらを前提とし，以下の処理を各時刻 $t$ で実行する．

### 1. Motion Update

各パーティクルを，運動モデルに従って前時刻から更新する．

$$
x_t^{(i)} \sim p(x_t \mid x_{t-1}^{(i)}, a_{t-1})
$$


### 2. Measurement Update

各パーティクルの観測の尤度 $p(o_t \mid x_t^{(i)})$ を計算し，
その値に基づいて重み $w_t^{(i)}$ を更新する．

$$
w_t^{(i)} \propto w_{t-1}^{(i)}\, p(o_t \mid x_t^{(i)}),
\qquad
p(o_t \mid x_t^{(i)})=\prod_{k=1}^{K} p(r_{t,k}\mid x_t^{(i)})
$$


### 3. Normalize

重みの総和が 1 になるように正規化する．

$$
w_t^{(i)} \leftarrow \frac{w_t^{(i)}}{\sum_{j=1}^{N} w_t^{(j)}}
$$


### 4. Resampling

重みの大きいパーティクルを残し，重みの小さいパーティクルを捨てるために，
重み $w_t^{(i)}$ に比例した確率で再び $N$ 個をサンプリングする．


### 5. Estimate

推定姿勢は，正規化された重みを用いた重み付き平均で求める．
ただし角度 $\theta$ は $2\pi$ 周期なので，循環平均を用いる．

$$
\hat x_t = \frac{\sum_i w_t^{(i)} x_t^{(i)}}{\sum_i w_t^{(i)}},\quad
\hat y_t = \frac{\sum_i w_t^{(i)} y_t^{(i)}}{\sum_i w_t^{(i)}}
$$

$$
\hat{\theta}_t
=
\operatorname{atan2}\!\bigl(
\sum_{i=1}^{N} w_t^{(i)}\sin\theta_t^{(i)},\;
\sum_{i=1}^{N} w_t^{(i)}\cos\theta_t^{(i)}
\bigr)
$$

## CSV log

実行すると `simple_mcl_log.csv` が生成されます．
各行には，`step`, `truth_x`, `truth_y`, `truth_theta`, `meas_0`, `meas_1`, `meas_2`, `est_x`, `est_y`, `est_theta` の順に値が記録されます．

+ `step`: ステップ番号
+ `truth_x`, `truth_y`, `truth_theta`: 姿勢の真値
+ `meas_0`, `meas_1`, `meas_2`: ランドマークの観測値
+ `est_x`, `est_y`, `est_theta`: 姿勢の推定値

## References

1. D. Fox, W. Burgard, F. Dellaert, and S. Thrun, “Monte Carlo Localization: Efficient Position Estimation for Mobile Robots,” *AAAI*, pp. 343–349, 1999.

1. 上田隆一, “[詳解 確率ロボティクス Pythonによる基礎アルゴリズムの実装](www.amazon.co.jp/dp/4065170060)”, 講談社, 2019.

1. 上田隆一, “[ロボットの確率・統計 製作・競技・知能研究で役立つ考え方と計算法](www.amazon.co.jp/dp/4339046876)”, コロナ社, 2024.

