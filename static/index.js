document.addEventListener('DOMContentLoaded', function () {
	const socket = io.connect(
		location.protocol + '//' + document.domain + ':' + location.port
	);
	socket.on('connect', () => {
		console.log('Connected to the Flask-SocketIO server');

		const ctx = document.getElementById('robotPositionChart').getContext('2d');
		const chartRange = 1000; // Define the range of the chart around the robot

		Chart.register(arrowPlugin);
		Chart.register(arcPlugin);

		const robotPositionChart = new Chart(ctx, {
			type: 'scatter',
			data: {
				datasets: [
					{
						label: 'Robot Position',
						backgroundColor: 'rgb(0, 0, 0)',
						data: [{ x: 250, y: 50 }], // Robot starts at the center
						heading: 90,
					},
					{
						label: 'Object Hits',
						backgroundColor: 'rgb(4, 59, 92)',
						data: [],
					},
				],
			},
			options: {
				plugins: {
					arcPlugin: {
						arcSpan: 0,
						direction: null,
					},
				},
				aspectRatio: 1,
				scales: {
					x: {
						type: 'linear',
						position: 'bottom',
						// will update dynamically
						min: 0,
						max: chartRange / 2,
					},
					y: {
						// will update dynamically
						min: 0,
						max: chartRange / 2,
					},
				},
			},
		});

		let lastUpdateTime = 0;
		const updateInterval = 100; // Time in milliseconds between updates

		function shouldUpdate() {
			const now = Date.now();
			if (now - lastUpdateTime > updateInterval) {
				lastUpdateTime = now;
				return true;
			}
			return false;
		}

		// Combined function to update both position and graph
		function updateRobotPositionAndGraphThrottled(posX, posY, heading) {
			if (!shouldUpdate()) return; // Skip update if interval hasn't passed

			// Update position and heading immediately
			const robotPosDataset = robotPositionChart.data.datasets[0];
			if (!posX && !posY && !heading) {
				const { x, y } = robotPosDataset.data[0];
				posX = x;
				posY = y;
				heading = robotPosDataset.heading;
			}
			robotPosDataset.data = [{ x: posX, y: posY }];
			robotPosDataset.heading = heading;

			robotPositionChart.update();

			// Delay graph update for visual effect, ensuring it's only scheduled once
			if (
				posX > robotPositionChart.options.scales.x.max ||
				posY > robotPositionChart.options.scales.y.max
			) {
				clearTimeout(robotPositionChart.updateTimeout);
				robotPositionChart.updateTimeout = setTimeout(() => {
					robotPositionChart.options.scales.x.max = posX + chartRange / 2;
					robotPositionChart.options.scales.y.max = posY + chartRange / 2;

					robotPositionChart.update();
				}, 300); // adjustable reposition time
			}
		}

		// throttled for now, probably not necessary?
		socket.on('pos_data', (data) => {
			console.log('Received pos_data:', data);
			updateRobotPositionAndGraphThrottled(data.x, data.y, data.heading);
		});

		// Smooth animation for the radar sweep
		function animateArcSpanChange(newArcSpan) {
			const startSpan = robotPositionChart.options.plugins.arcPlugin.arcSpan;
			const change = newArcSpan - startSpan;
			const duration = 500; // Duration of the animation in milliseconds
			const startTime = Date.now();

			function animate() {
				const currentTime = Date.now();
				const timeElapsed = currentTime - startTime;
				const progress = Math.min(timeElapsed / duration, 1); // Ensure progress doesn't exceed 1

				// Update arcSpan based on the current progress
				robotPositionChart.options.plugins.arcPlugin.arcSpan =
					startSpan + change * progress;
				robotPositionChart.update();

				if (progress < 1) {
					requestAnimationFrame(animate); // Continue the animation if not yet completed
				}
			}

			requestAnimationFrame(animate); // Start the animation
		}

		// Listen for sweep data from the socket
		socket.on('sweep_data', (data) => {
			console.log('Received sweep_data:', data);

			if (!robotPositionChart.config.options.plugins.arcPlugin.direction) {
				robotPositionChart.config.options.plugins.arcPlugin.direction =
					data.sweepDegree < 40 ? 'right' : 'left';
			}
			const direction = robotPositionChart.config.options.plugins.arcPlugin.direction;

			const adjustedDegree =
				direction === 'right'
					? data.sweepDegree - 30
					: Math.abs(data.sweepDegree - 180 + 30);
			animateArcSpanChange(adjustedDegree);

			if (data.sweepHit > 1 && data.sweepHit <= 70) {
				console.log('valid hit');

				const robotPosition = robotPositionChart.data.datasets[0].data[0]; // Current position of the robot

				console.log('Adjusted degree:', adjustedDegree);

				const heading = robotPositionChart.data.datasets[0].heading;

				// Convert heading to radians for trigonometric functions
				const headingRadians = (heading * Math.PI) / 180; // Adjust by -90 degrees because 0 radians points right
				const adjustedDegreeRadians = (adjustedDegree * Math.PI) / 180;

				// Calculate the direction of the hit relative to the robot's heading
				const hitDirectionRadians = headingRadians + adjustedDegreeRadians - Math.PI / 3; // Subtract PI/3 to adjust for arc starting point

				// Calculate hit position
				const hitX = robotPosition.x + data.sweepHit * Math.cos(hitDirectionRadians);
				const hitY = robotPosition.y + data.sweepHit * Math.sin(hitDirectionRadians);

				// Push the calculated hit position to the dataset for object hits.
				robotPositionChart.data.datasets[1].data.push({ x: hitX, y: hitY });

				console.log('Hit position relative to robot:', hitX, hitY);
			}

			// Redraw the chart to reflect the updated arc
			robotPositionChart.update();
		});
	});

	const btn = document.getElementById('tester');

	// mock server receiving bot data event, normally this comes from the server not the client
	btn.onclick = () => {
		socket.emit('trigger_data', {
			x: 300, // Random X within fixed range for demonstration
			y: 150, // Random Y within fixed range for demonstration
			heading: 120,
		});
		console.log('Emitted test data (client send)');
	};

	const sweep = document.getElementById('sweep');
	// ex: debug sweep
	sweep.onclick = () => {
		let i = 30;
		const sweepIntervalForward = setInterval(() => {
			socket.emit('sweep_data', {
				sweepDegree: i, // Random X within fixed range for demonstration
				sweepHit: Math.floor(Math.random() * (70 - 50 + 1)) + 50,
			});
			if (i === 150) {
				clearInterval(sweepIntervalForward);
				console.log('sweep forward end');
			} else i += 5;
		}, 50);
		console.log('Emitted test data (client send)');
	};

	const sweepBack = document.getElementById('sweepBack');

	sweepBack.onclick = () => {
		let i = 150;
		const sweepIntervalBackward = setInterval(() => {
			socket.emit('sweep_data', {
				sweepDegree: i, // Random X within fixed range for demonstration
				sweepHit: Math.floor(Math.random() * (70 - 50 + 1)) + 50,
			});
			if (i === 30) {
				clearInterval(sweepIntervalBackward);
				console.log('sweep backward end');
			} else i -= 5;
		}, 50);
		console.log('sweep backward begin');
	};
});

// arc plugin
const arcPlugin = {
	id: 'arcPlugin',
	afterDatasetsDraw: function (chart, easing, options) {
		const ctx = chart.ctx;
		const meta = chart.getDatasetMeta(0); // Get metadata for the first dataset
		const point = meta.data[0]; // Assuming only 1 point exists in the dataset
		const heading = 360 - chart.data.datasets[0].heading; // Input heading in degrees
		const arcSpan = chart.config.options.plugins.arcPlugin.arcSpan; // Default arcSpan is 120 if not specified
		const direction = chart.config.options.plugins.arcPlugin.direction; // Default arcSpan is 120 if not specified
		const radius = 140;

		// thanks trigonometry lessons
		// Convert heading and angles to radians
		const headingRadians = (Math.PI / 180) * heading;

		let startAngle, endAngle;

		ctx.beginPath();
		ctx.moveTo(point.x, point.y); // Move to the center of the circle
		ctx.fillStyle = 'rgba(255, 0, 0, 0.2)'; // Set fill color

		if (direction === 'right') {
			startAngle = headingRadians - Math.PI / 3;
			endAngle = startAngle + (Math.PI / 180) * arcSpan;

			ctx.arc(point.x, point.y, radius, startAngle, endAngle);
		} else if (direction === 'left') {
			startAngle = headingRadians + Math.PI / 3;
			endAngle = startAngle - (Math.PI / 180) * arcSpan;

			ctx.arc(point.x, point.y, radius, endAngle, startAngle, false);
		}

		if (arcSpan >= 120) {
			chart.config.options.plugins.arcPlugin.arcSpan = 0;
			chart.config.options.plugins.arcPlugin.direction = null;
		}

		ctx.fill();
		ctx.closePath();
	},
};

// heading arrow
const arrowPlugin = {
	id: 'arrowPlugin',
	afterDatasetsDraw: function (chart, easing) {
		if (chart.config.type !== 'scatter') return; // Ensure this is a scatter chart

		const ctx = chart.ctx;
		const dataset = chart.data.datasets[0]; // Assuming the robot's position is in the first dataset
		if (!dataset.data.length) return;

		const point = chart.getDatasetMeta(0).data[0].getCenterPoint(); // Get the center point of the robot's position
		const heading = dataset.heading || 0; // Assuming you store the heading in the dataset

		// Calculate the arrow's drawing coordinates
		const arrowLength = 20; // Adjust the length of the arrow as needed
		const endX = point.x + arrowLength * Math.cos((Math.PI * heading) / 180);
		const endY = point.y - arrowLength * Math.sin((Math.PI * heading) / 180); // Subtracting because the y-axis is inverted

		// Draw the arrow line
		ctx.save();
		ctx.beginPath();
		ctx.moveTo(point.x, point.y);
		ctx.lineTo(endX, endY);
		ctx.strokeStyle = 'red'; // Arrow color
		ctx.lineWidth = 2;
		ctx.stroke();

		// Draw the arrowhead
		const arrowheadLength = 10;
		const angle = Math.atan2(endY - point.y, endX - point.x);
		ctx.beginPath();
		ctx.moveTo(endX, endY);
		ctx.lineTo(
			endX - arrowheadLength * Math.cos(angle - Math.PI / 6),
			endY - arrowheadLength * Math.sin(angle - Math.PI / 6)
		);
		ctx.lineTo(
			endX - arrowheadLength * Math.cos(angle + Math.PI / 6),
			endY - arrowheadLength * Math.sin(angle + Math.PI / 6)
		);
		ctx.lineTo(endX, endY);
		ctx.lineTo(
			endX - arrowheadLength * Math.cos(angle - Math.PI / 6),
			endY - arrowheadLength * Math.sin(angle - Math.PI / 6)
		); // This line is needed to create a filled arrowhead with a proper outline
		ctx.fillStyle = 'red';
		ctx.fill();
		ctx.strokeStyle = 'red';
		ctx.stroke();

		ctx.restore();
	},
};
