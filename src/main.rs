#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use std::time::{Duration, Instant};

use ::futures::StreamExt;
use eframe::egui::{Color32, ComboBox, Slider, Vec2, Vec2b};
use eframe::{egui, CreationContext};
use egui_plot::{Arrows, CoordinatesFormatter, PlotPoints, Points};
use ld19codec::{Ld19Frame, Ld19Point};
use tokio::runtime;

use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::Decoder;

mod ld19codec;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1024.0, 768.0])
            .with_min_inner_size([800.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "LD19 LIDAR Viewer",
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

#[derive(Debug, Default)]
struct LidarStats {
    angular_resolution: RollingAverage,
    angular_rate: RollingAverage,
    sample_rate: RollingAverage,
    max_dist: RollingAverage,
    min_dist: RollingAverage,
    crc_errors: u32,
    last_start_angle: f32,
    last_completed_rotation: Option<Instant>,
}

#[derive(Debug, Default)]
struct RollingAverage {
    index: usize,
    hist: [f32; 8],
}

impl RollingAverage {
    fn push(&mut self, val: f32) {
        self.hist[self.index] = val;
        self.index = (self.index + 1) % self.hist.len();
    }

    fn get(&self) -> f32 {
        self.hist.iter().sum::<f32>() / self.hist.len() as f32
    }
}

struct ViewerApp {
    rt: runtime::Runtime,
    lidar_rx: Option<std::sync::mpsc::Receiver<Ld19Frame>>,
    lidar_points: Vec<LidarPoint>,
    intensity_threshold: f32,
    fade_duration_ms: u64,
    serial_port: String,
    worker_handle: Option<tokio::task::JoinHandle<()>>,
    stop_signal: Option<tokio::sync::mpsc::Sender<()>>,
    stats: LidarStats,
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
            intensity_threshold: 0.1,
            fade_duration_ms: 100, // 10Hz
            serial_port: "".to_owned(),
            worker_handle: None,
            stop_signal: None,
            stats: Default::default(),
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("options").show(ctx, |ui| {
            // device disconnected?
            if self
                .stop_signal
                .as_ref()
                .map(|p| p.is_closed())
                .unwrap_or(false)
            {
                self.worker_handle = None;
                self.stop_signal = None;
                self.serial_port = String::new();
            }

            ui.style_mut().spacing.item_spacing = Vec2::new(16.0, 16.0);
            ui.style_mut().spacing.indent = 16.0;

            ui.add_space(ui.spacing().item_spacing.y);
            ui.spacing();
            ui.heading("Settings");
            ComboBox::from_label("Serial port")
                .selected_text(self.serial_port.to_string())
                .show_ui(ui, |ui| {
                    for port in available_serial_ports() {
                        let resp =
                            ui.selectable_value(&mut self.serial_port, port.clone(), port.clone());

                        if resp.changed() {
                            // exit the worker task
                            if self.worker_handle.as_ref().is_some() {
                                self.rt
                                    .block_on(self.stop_signal.as_ref().unwrap().send(()))
                                    .ok();
                                self.worker_handle = None;
                                self.stop_signal = None;
                            }

                            // create a new worker
                            let (tx, rx) = std::sync::mpsc::channel();
                            self.lidar_rx = Some(rx);

                            let egui_ctx = ctx.clone();

                            let (tx_stop, mut rx_stop) = tokio::sync::mpsc::channel(1);
                            self.stop_signal = Some(tx_stop);

                            self.worker_handle = Some(self.rt.spawn(async move {
                                let port = tokio_serial::new(port, 230400)
                                    .stop_bits(tokio_serial::StopBits::One)
                                    .parity(tokio_serial::Parity::None)
                                    .flow_control(tokio_serial::FlowControl::None)
                                    .open_native_async()
                                    .expect("Cannot open port");

                                let codec = ld19codec::Ld19Codec {};
                                let mut reader = codec.framed(port);

                                loop {
                                    tokio::select! {
                                        Some(frame) = reader.next() => {
                                            if let Ok(result) = frame {
                                                tx.send(result).unwrap();
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

                                println!("exit worker");
                            }));

                            // clear plot and reset stats
                            self.lidar_points.clear();
                            self.stats = Default::default();
                        }
                    }
                });

            ui.add(
                Slider::new(&mut self.intensity_threshold, 0.0..=1.0).text("Intensity threshold"),
            );
            ui.add(Slider::new(&mut self.fade_duration_ms, 0..=500).text("Fade duration (ms)"))
                .on_hover_ui(|ui| {
                    ui.label("This is typically the angular frequency (100ms for the LD19)");
                });

            // stats ui
            ui.separator();
            ui.heading("Stats");
            egui::Grid::new("stats")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    ui.label("CRC errors");
                    ui.label(format!("{}", self.stats.crc_errors));
                    ui.end_row();
                    ui.label("Sample rate");
                    ui.label(format!("{:.1}kHz", self.stats.sample_rate.get() * 1e-3));
                    ui.end_row();
                    ui.label("Angular rate");
                    ui.label(format!("{:.1}Hz", self.stats.angular_rate.get()));
                    ui.end_row();
                    ui.label("Angular resolution");
                    ui.label(format!("{:.2}°", self.stats.angular_resolution.get()));
                    ui.end_row();
                    ui.label("Min distance");
                    ui.label(format!("{:.2}m", self.stats.min_dist.get()));
                    ui.end_row();
                    ui.label("Max distance");
                    ui.label(format!("{:.2}m", self.stats.max_dist.get()));
                });

            // fetch new datapoints
            if let Some(lidar_rx) = self.lidar_rx.as_ref() {
                while let Ok(frame) = lidar_rx.try_recv() {
                    match frame {
                        Ld19Frame::Packet(packet) => {
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

                            // calculate stats
                            self.stats
                                .angular_resolution
                                .push(packet.delta_angle_per_point_deg());
                            if self.stats.last_start_angle > packet.start_angle_deg() {
                                let dt = Instant::now().duration_since(
                                    self.stats.last_completed_rotation.unwrap_or(Instant::now()),
                                );
                                self.stats.last_completed_rotation = Some(Instant::now());
                                self.stats.angular_rate.push(dt.as_secs_f32().recip());
                                self.stats.sample_rate.push(
                                    dt.as_secs_f32().recip()
                                        * (360.0 / packet.delta_angle_per_point_deg()),
                                );
                            }
                            self.stats.last_start_angle = packet.start_angle_deg();
                            self.stats.max_dist.push(
                                self.lidar_points
                                    .iter()
                                    .max_by(|a, b| {
                                        a.point
                                            .distance_in_meters()
                                            .total_cmp(&b.point.distance_in_meters())
                                    })
                                    .map(|p| p.point.distance_in_meters())
                                    .unwrap_or_default(),
                            );
                            self.stats.min_dist.push(
                                self.lidar_points
                                    .iter()
                                    .min_by(|a, b| {
                                        a.point
                                            .distance_in_meters()
                                            .total_cmp(&b.point.distance_in_meters())
                                    })
                                    .map(|p| p.point.distance_in_meters())
                                    .unwrap_or_default(),
                            );
                        }
                        Ld19Frame::CRCError => self.stats.crc_errors += 1,
                    }
                }
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            if self.serial_port.is_empty() {
                ui.vertical_centered(|ui| {
                    ui.add_space(ui.available_height() * 0.5);
                    ui.heading("LIDAR not connected");
                    ui.label("Connect your LIDAR device and select a serial port")
                });
            } else {
                egui_plot::Plot::new("plot")
                    .allow_zoom(true)
                    .allow_drag(true)
                    .allow_scroll(false)
                    .auto_bounds(Vec2b::new(false, false))
                    .x_axis_label("m")
                    .y_axis_label("m")
                    .data_aspect(1.0)
                    .coordinates_formatter(
                        egui_plot::Corner::LeftBottom,
                        CoordinatesFormatter::new(|p, _| {
                            let d = (p.x * p.x + p.y * p.y).sqrt();
                            let ang = p.x.atan2(p.y).to_degrees();
                            format!("d={:.2}m θ={:.2}°", d, ang)
                        }),
                    )
                    .show(ui, |plot_ui| {
                        let points: Vec<_> = self
                            .lidar_points
                            .iter()
                            .map(|p| {
                                let rad = p.angle.to_radians();

                                // align +y with the forward direction of the sensor
                                let x = rad.sin() * p.point.distance_in_meters();
                                let y = rad.cos() * p.point.distance_in_meters();

                                [x as f64, y as f64]
                            })
                            .collect();

                        let plot_points = Points::new(points).radius(2.5).color(Color32::GREEN);
                        plot_ui.points(plot_points);
                        plot_ui.arrows(
                            Arrows::new(
                                PlotPoints::new(vec![[0.0, 0.0]]),
                                PlotPoints::new(vec![[0.0, 1.0]]),
                            )
                            .allow_hover(false),
                        );

                        let plot_points = Points::new(vec![[0.0, 0.0]])
                            .radius(10.0)
                            .color(Color32::GOLD);
                        plot_ui.points(plot_points);
                    });
            }
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
