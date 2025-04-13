function updateStatus() {
    fetch('/status')
      .then(res => res.json())
      .then(data => {
        const stackBox = document.getElementById('stack-status');
        stackBox.innerHTML = `Stack Light: <b>${data.stack}</b>`;
        stackBox.className = 'status-box ' + (data.stack == 0 ? 'green' : data.stack == 1 ? 'yellow' : 'red');
  
        document.getElementById('door-status').innerHTML = `Door State: <b>${data.door}</b>`;
        document.getElementById('emergency-status').innerHTML = `Emergency Button: <b>${data.emergency}</b>`;
        document.getElementById('barcode').innerHTML = `Last Scanned Barcode: <b>${data.barcode}</b>`;
      });
  }
  
  function toggleDoor() {
    fetch('/toggle-door', { method: 'POST' })
      .then(res => res.json())
      .then(data => alert(data.result));
  }
  
  function pressEmergency() {
    fetch('/press-emergency', { method: 'POST' })
      .then(res => res.json())
      .then(data => alert(data.result));
  }
  
  function releaseEmergency() {
    fetch('/release-emergency', { method: 'POST' })
      .then(res => res.json())
      .then(data => alert(data.result));
  }
  
  function sendPickRequest() {
    const pickId = document.getElementById('pickId').value;
    const quantity = document.getElementById('quantity').value;
  
    fetch('http://ip_address:8080/pick', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pickId: Number(pickId), quantity: Number(quantity) })
    })
    .then(res => res.json())
    .then(data => {
      const output = `
        <b>Pick ID:</b> ${data.pickId} <br>
        <b>Status:</b> ${data.pickSuccessful ? "✅ SUCCESS" : "❌ FAILED"} <br>
        <b>Barcode:</b> ${data.itemBarcode || "—"} <br>
        <b>Message:</b> ${data.errorMessage || "No error"}
      `;
      document.getElementById('pick-response').innerHTML = output;
    })
    .catch(err => {
      document.getElementById('pick-response').innerHTML = "<b>Error sending pick request</b>";
      console.error(err);
    });
  }
  
  setInterval(updateStatus, 2000);
  updateStatus();
