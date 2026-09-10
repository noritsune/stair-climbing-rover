// Page served at http://192.168.4.1/ when the camera failed to initialise.
//
// The board raises its access point and both HTTP servers before touching the
// camera, so a dead or unplugged camera no longer takes the whole board down
// with it. This page is what the viewer page is replaced by in that case: it
// names the error code and the usual causes, so a fault can be diagnosed from
// the phone that is already connected instead of a USB cable and a laptop.
//
// Split in two because the error detail is inserted between the halves at send
// time; the CSS below contains '%', which rules out formatting it with printf.

#pragma once

static const char CAMERA_ERROR_HTML_HEAD[] = R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover Camera - カメラエラー</title>
<style>
  body { margin: 0; min-height: 100dvh; background: #111; color: #eee;
         font-family: sans-serif;
         display: flex; align-items: center; justify-content: center; }
  main { max-width: 30rem; padding: 1.5rem; }
  h1 { font-size: 1.2rem; margin: 0 0 0.8rem; color: #ff8a80; }
  p { line-height: 1.6; }
  code { display: block; margin: 1rem 0; padding: 0.6rem 0.8rem;
         background: #000; border: 1px solid #444; border-radius: 4px;
         word-break: break-all; }
  ul { padding-left: 1.2rem; line-height: 1.8; }
  .note { color: #aaa; font-size: 0.85rem; }
  button { margin-top: 0.5rem; padding: 0.6rem 1.2rem; font-size: 1rem;
           background: #333; color: #eee; border: 1px solid #666;
           border-radius: 4px; }
</style>
</head>
<body>
<main>
<h1>カメラを初期化できませんでした</h1>
<p>Wi-Fi と Web サーバーは動作しています。カメラモジュールだけが応答していません。</p>
<code>)rawliteral";

static const char CAMERA_ERROR_HTML_TAIL[] = R"rawliteral(</code>
<ul>
  <li>リボンケーブルが奥まで挿さっているか、裏返しになっていないか</li>
  <li>コネクタのロックが下りているか</li>
  <li>電源が足りているか（5V / 500mA 以上）</li>
  <li>一度電源を切って入れ直す</li>
</ul>
<p class="note">直らない場合はシリアルモニタ（115200 bps）の起動ログを確認してください。
カメラ以外は正常に立ち上がっているので、この画面が出ている時点で Wi-Fi 側の問題ではありません。</p>
<button onclick="location.reload()">再読み込み</button>
</main>
</body>
</html>
)rawliteral";
