/**
 * phone-robot 走行記録 自動 Google ドライブ保存用 Google Apps Script (Webhook)
 *
 * 【セットアップ手順（画面操作なしでドライブへ自動保存したい場合のみ）】
 * 1. Google ドライブで「新規」→「その他」→「Google Apps Script」を開く。
 * 2. このファイルの中身をすべて貼り付けて保存する。
 * 3. 右上の「デプロイ」→「新しいデプロイ」→ 種類の選択で「ウェブアプリ」を選ぶ。
 * 4. 次のユーザーとして実行：「自分」、アクセスできるユーザー：「全員」にしてデプロイする。
 *    （初回のみ自分の Google アカウントへのドライブ権限許可画面が出ます）
 * 5. 発行された「ウェブアプリ URL」（https://script.google.com/macros/s/.../exec）をコピーし、
 *    phone-robot アプリの「URL設定」画面に貼り付けて保存する。
 *
 * ※ URL を設定しなくても、アプリは端末の Download/PhoneRobot と Google ドライブ共有シート経由で保存できます。
 */
function doPost(e) {
  try {
    if (!e || !e.postData || !e.postData.contents) {
      return jsonResponse({ ok: false, error: 'empty body' });
    }
    var payload = JSON.parse(e.postData.contents);
    var runId = String(payload.runId || 'unknown').replace(/[^A-Za-z0-9_-]/g, '');
    var files = payload.files || [];

    var rootName = 'phone-robot_logs';
    var rootIter = DriveApp.getFoldersByName(rootName);
    var rootFolder = rootIter.hasNext() ? rootIter.next() : DriveApp.createFolder(rootName);

    var runFolderName = 'run_' + runId;
    var runIter = rootFolder.getFoldersByName(runFolderName);
    var runFolder = runIter.hasNext() ? runIter.next() : rootFolder.createFolder(runFolderName);

    var savedNames = [];
    for (var i = 0; i < files.length; i++) {
      var item = files[i];
      if (!item || !item.name || !item.base64) continue;
      var safeName = String(item.name).replace(/[\/\\]/g, '_');
      var mimeType = item.mimeType || 'application/octet-stream';
      var bytes = Utilities.base64Decode(item.base64);
      var blob = Utilities.newBlob(bytes, mimeType, safeName);

      var existing = runFolder.getFilesByName(safeName);
      while (existing.hasNext()) {
        existing.next().setTrashed(true);
      }
      runFolder.createFile(blob);
      savedNames.push(safeName);
    }

    return jsonResponse({
      ok: true,
      runId: runId,
      savedCount: savedNames.length,
      folderUrl: runFolder.getUrl(),
    });
  } catch (err) {
    return jsonResponse({ ok: false, error: String(err) });
  }
}

function jsonResponse(obj) {
  return ContentService.createTextOutput(JSON.stringify(obj))
    .setMimeType(ContentService.MimeType.JSON);
}
