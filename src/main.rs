use std::time::{Duration, Instant};

use ::futures::StreamExt;
use eframe::egui::{ComboBox, Slider, Vec2b};
use eframe::{egui, CreationContext};
use egui_plot::{Arrows, PlotPoints, Points};
use ld19codec::{Ld19Packet, Ld19Point};
use tokio::runtime;

use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::Decoder;
mod ld19codec;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1024.0, 768.0]),
        ..Default::default()
    };
    eframe::run_native(
        "LD19 Lidar Viewer",
        options,
        Box::new(|cc| Ok(Box::new(ViewerApp::new(cc)))),
    )
}

#[derive(Clone, Copy)]
struct LidarPoint {
    point: Ld19Point,
    angle: f32,
    instant: Instant,
}

struct ViewerApp {
    rt: runtime::Runtime,
    lidar_rx: Option<std::sync::mpsc::Receiver<Ld19Packet>>,
    lidar_points: Vec<LidarPoint>,
    intensity_threshold: f32,
    fade_duration_ms: u64,
    serial_port: String,
    worker_handle: Option<tokio::task::JoinHandle<()>>,
    stop_signal: Option<tokio::sync::mpsc::Sender<()>>,
}

impl ViewerApp {
    fn new(_ctx: &CreationContext) -> Self {
        Self {
            rt: runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap(),
            lidar_rx: None,
            lidar_points: vec![],
            intensity_threshold: 0.5,
            fade_duration_ms: 100, // 10Hz
            serial_port: "".to_owned(),
            worker_handle: None,
            stop_signal: None,
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // egui::CentralPanel::default().show(ctx, |ui| {
        egui::SidePanel::left("options").show(ctx, |ui| {
            ui.heading("Settings");
            ComboBox::from_label("Serial port")
                .selected_text(self.serial_port.to_string())
                .show_ui(ui, |ui| {
                    for port in available_serial_ports() {
                        let resp =
                            ui.selectable_value(&mut self.serial_port, port.clone(), port.clone());

                        if resp.clicked() {
                            // exit the worker task
                            if self.worker_handle.as_ref().is_some() {
                                self.rt
                                    .block_on(self.stop_signal.as_ref().unwrap().send(()))
                                    .unwrap();
                                self.worker_handle = None;
                                self.stop_signal = None;
                            }

                            // create a new worker
                            let (tx, rx) = std::sync::mpsc::channel();
                            self.lidar_rx = Some(rx);

                            let egui_ctx = ctx.clone();

                            let (tx_stop, mut rx_stop) = tokio::sync::mpsc::channel(10);
                            self.stop_signal = Some(tx_stop);

                            self.worker_handle = Some(self.rt.spawn(async move {
                                let port = tokio_serial::new(port, 230400)
                                    .stop_bits(tokio_serial::StopBits::One)
                                    .parity(tokio_serial::Parity::None)
                                    .open_native_async()
                                    .expect("Cannot open port");

                                let codec = ld19codec::Ld19Codec {};
                                let mut reader = codec.framed(port);

                                loop {
                                    tokio::select! {
                                        Some(packet) = reader.next() => {
                                            if let Ok(packet) = packet {
                                                tx.send(packet).unwrap();
                                                egui_ctx.request_repaint();
                                            } else {
                                                break;
                                            }
                                        },

                                        Some(_) = rx_stop.recv() => {
                                            break;
                                        }
                                    }
                                }

                                println!("exit");
                            }));

                            // clear plot
                            self.lidar_points.clear();
                        }
                    }
                });

            ui.add(
                Slider::new(&mut self.intensity_threshold, 0.0..=1.0).text("Intensity threshold"),
            );
            ui.add(Slider::new(&mut self.fade_duration_ms, 0..=500).text("Fade duration (ms)"));

            // fetch new datapoints
            if let Some(lidar_rx) = self.lidar_rx.as_ref() {
                while let Ok(packet) = lidar_rx.try_recv() {
                    let fade_dur = Duration::from_millis(self.fade_duration_ms);

                    for point_angle in packet.iter_points() {
                        self.lidar_points.push(LidarPoint {
                            point: *point_angle.1,
                            angle: point_angle.0,
                            instant: Instant::now(),
                        });
                    }

                    // filter datapoints
                    let points: Vec<_> = self
                        .lidar_points
                        .iter()
                        .filter(|p| {
                            Instant::now().duration_since(p.instant) < fade_dur
                                && p.point.normalized_intensity() > self.intensity_threshold
                        })
                        .copied()
                        .collect();

                    self.lidar_points = points;
                }
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui_plot::Plot::new("plot")
                .allow_zoom(true)
                .allow_drag(true)
                .allow_scroll(false)
                .auto_bounds(Vec2b::new(false, false))
                .show(ui, |plot_ui| {
                    let points: Vec<_> = self
                        .lidar_points
                        .iter()
                        .map(|p| {
                            let rad = p.angle / 180.0 * std::f32::consts::PI;

                            // align +y with the forward direction of the sensor
                            let x = rad.sin() * p.point.distance_in_meters();
                            let y = rad.cos() * p.point.distance_in_meters();

                            [x as f64, y as f64]
                        })
                        .collect();

                    let plot_points = Points::new(points).radius(2.5);
                    plot_ui.points(plot_points);
                    plot_ui.arrows(Arrows::new(
                        PlotPoints::new(vec![[0.0, 0.0]]),
                        PlotPoints::new(vec![[0.0, 1.0]]),
                    ));
                });
        });
    }
}

fn available_serial_ports() -> Vec<String> {
    if let Ok(ports) = tokio_serial::available_ports() {
        // Note: filtering on port type does not work / is unreliable
        return ports
            .iter()
            .map(|p| p.port_name.clone())
            .filter(|p| !p.contains("ttyS"))
            .collect();
    }

    vec![]
}