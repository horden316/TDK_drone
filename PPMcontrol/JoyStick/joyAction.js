var http = require('http'); 

var server = http.createServer(function (req, res) {   // 2 - 建立server
 
    // 在此處理 客戶端向 http server 發送過來的 req。
 
});
 
server.listen(5000);
 
console.log('Node.js web server at port 5000 is running..')

while (true) {
    var X1value = get1X();
    var Y1value = get1Y();
    var X2value = get2X();
    var Y2value = get2Y();

    console.log(X1value, Y1value, X2value, Y2value);
}