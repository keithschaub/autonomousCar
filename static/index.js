document.addEventListener('DOMContentLoaded', function () {
	const socket = io.connect(
		location.protocol + '//' + document.domain + ':' + location.port
	);
	socket.on('connect', () => {
		console.log('Connected to the Flask-SocketIO server');

		// handle visuals based off client receiving this!
		socket.on('pos_data', (data) => {
			console.log('Received pos_data:', data);
		});
	});

	const btn = document.getElementById('tester');

	// mock server receiving bot data event, normally this comes from the server not the client
	btn.onclick = () => {
		socket.emit('trigger_data', {
			x: 0,
			y: 0,
			diff: 0,
			sweep: [],
		});
		console.log('Emitted test data');
	};
});
