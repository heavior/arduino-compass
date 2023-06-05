const http = require('http');
const fs = require('fs');
const path = require('path');

const server = http.createServer((req, res) => {
  const filePath = path.join(__dirname, req.url); // Resolve the file path based on the requested URL
  const fileStream = fs.createReadStream(filePath);

  fileStream.on('error', () => {
    // If the file doesn't exist or there was an error reading it, return a 404 response
    res.writeHead(404, { 'Content-Type': 'text/plain' });
    res.end('File not found');
  });

  fileStream.pipe(res); // Stream the file contents to the response
});

const port = 3000;
server.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});
