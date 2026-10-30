# racegame

鈴鹿サーキットを走る、3D レースゲームのひな形です。車は 1 台です。ドリフトは入っていません。

コースの線形は OpenStreetMap（relation 284570、Suzuka International Racing Course）、高さは国土地理院の標高タイルです。

- 地図データ: © OpenStreetMap contributors
- 標高: 出典 国土地理院

画像は `assets/graphics/`、音声は `assets/sound/` に置きます。コースデータは `assets/track/suzuka.txt` です。

## 開き方

Cursor または VS Code でこのフォルダを開き、**Reopen in Container** を選びます。初回はイメージのビルドに数分かかります。

## 操作

キーボードとゲームパッドは、一般的な 3D レースゲームと同じです。ハンドブレーキはありません。

| 操作 | キーボード | ゲームパッド |
|---|---|---|
| アクセル | W / ↑ | RT（R2）。十字キー上でも可 |
| ブレーキ / 後退 | S / ↓ | LT（L2）。十字キー下でも可 |
| ステアリング | A D / ← → | 左スティック、十字キー左右 |
| リスタート | R | Back（View / Share） |
| カメラ | C | Y（△） |
| ポーズ | P | Start（Menu / Options） |
| 操作表示 | H | |

画面は追従、近め、遠めの 3 つです。ギアはオートマです。

## 音

エンジン音、タイヤのスキール音、BGM（タイトル用とプレイ中用の 2 曲）、ラップの音、ガードレールに当たる音、横転の音を鳴らします。

- エンジン音は回転数とアクセルで高さと明るさ、大きさが変わります。スキール音はドリフト中に、滑る角度と速さで大きくなります。
- BGM はシーンが切り替わると、前の曲を下げてから次の曲に変わります。
- ラップの音は 1 周ごとに鳴り、ベストラップを縮めたときは別の音になります。
- ガードレールや壁に当たると、当たりの強さに合わせた音が鳴ります。車体が横転すると、転がる音が鳴ります。
- 音量はプレイ中にポーズの画面（P / Start）で BGM と効果音を変えられます。ポーズの画面からタイトル（シーン選択）にも戻れます。
  試験コースの調整の画面（Tab）の SOUND の項目では、全体の音量も変えられます。
- ブラウザ版は自動再生の制限があるので、最初のキー入力かクリックで音が始まります。
- 音の出ない環境では音を出さずにそのまま動きます。

音はすべて `src/System/SoundSynth.cpp` で実行時に合成しています。音のファイルや外部の素材は使っていません。
BGM の 2 曲もこのリポジトリのために書いたもので、このリポジトリのコードと同じ扱いです。

## ドリフトの課題

滑らない動きは `Car::stepGrip`（`src/car.cpp`）だけです。速度は車体の前を向き、横滑りはしません。ドリフトはこの関数を差し替えて作ります。アクセル、ブレーキ、壁、路面の高さは `Car::update` 側です。

## ブラウザ版

main の最新版を https://huu726.github.io/Drift/ で遊べます。main に push すると `.github/workflows/pages.yml` が自動で公開し直します。

手元で動かすときは次の手順です。

```bash
emcmake cmake -B build
cmake --build build
python3 -m http.server 8080
```

`http://localhost:8080/build/racegame.html`

## Linux 版

```bash
cmake -B build-native
cmake --build build-native
./build-native/racegame
```

画面は `http://localhost:6080`（パスワード `vscode`）です。コンテナ内では音は鳴りません（音が開けないとログに出し、そのまま動きます）。

ロジックだけの確認:

```bash
./build-native/racegame --self-test
```
