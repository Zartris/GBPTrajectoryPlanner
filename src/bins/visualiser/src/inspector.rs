// src/bins/visualiser/src/inspector.rs
//
//! InspectorPlugin — click-to-inspect for per-variable GBP diagnostics.
//!
//! # Usage
//!
//! - Each per-robot egui panel has an "Inspect" button. Clicking it sends an
//!   `inspect` command to the simulator requesting variable 0 (the current
//!   position variable).
//! - The simulator responds with an `inspect_response` JSON message containing
//!   mean, variance, eta, lambda, and per-factor message summaries.
//! - `InspectorPlugin` drains `InspectInbox`, logs results with `tracing::info!`,
//!   and stores the latest result in `LastInspectResult` resource for display.
//! - A floating egui popup ("Variable Inspector") shows the last result.
//!
//! # Wire format
//!
//! Request (sent by UI via WsOutbox):
//! ```json
//! {"command":"inspect","robot_id":0,"variable_k":0}
//! ```
//!
//! Response (received via InspectInbox):
//! ```json
//! {"type":"inspect_response","robot_id":0,"variable_k":0,
//!  "mean":2.5,"variance":0.01,
//!  "factors":[{"kind":"Dynamics","msg_eta":0.1,"msg_lambda":5.0}]}
//! ```

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::{InspectInbox, InspectResponse};

// ── Resource ─────────────────────────────────────────────────────────────────

/// Stores the most recently received inspect response for display in the popup.
#[derive(Resource, Default)]
pub struct LastInspectResult(pub Option<InspectResponse>);

// ── Plugin ───────────────────────────────────────────────────────────────────

pub struct InspectorPlugin;

impl Plugin for InspectorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LastInspectResult>()
            .add_systems(Update, drain_inspect_inbox)
            .add_systems(EguiPrimaryContextPass, draw_inspect_popup);
    }
}

// ── Systems ───────────────────────────────────────────────────────────────────

/// Drain InspectInbox: log each response and store the latest in LastInspectResult.
fn drain_inspect_inbox(
    inbox: Res<InspectInbox>,
    mut last: ResMut<LastInspectResult>,
) {
    let mut q = inbox.0.lock().unwrap_or_else(|e| e.into_inner());
    while let Some(resp) = q.pop_front() {
        // Terminal dump
        tracing::info!(
            "[inspect] robot={} k={}: mean={:.4} variance={:.4} factors={}",
            resp.robot_id,
            resp.variable_k,
            resp.mean,
            resp.variance,
            resp.factors.len(),
        );
        for (i, f) in resp.factors.iter().enumerate() {
            tracing::info!(
                "[inspect]   factor[{}] kind={} eta={:.4} lambda={:.4}",
                i, f.kind, f.msg_eta, f.msg_lambda,
            );
        }
        last.0 = Some(resp);
    }
}

/// Floating egui popup showing the last inspect response.
fn draw_inspect_popup(
    mut ctxs: EguiContexts,
    last: Res<LastInspectResult>,
    mut open: Local<bool>,
) -> Result {
    let ctx = ctxs.ctx_mut()?;

    // Show popup if we have a result; auto-open when new result arrives.
    if last.is_changed() && last.0.is_some() {
        *open = true;
    }

    if !*open {
        return Ok(());
    }

    let Some(ref resp) = last.0 else {
        return Ok(());
    };

    let heading = egui::Color32::from_rgb(200, 210, 230);
    let accent  = egui::Color32::from_rgb(100, 200, 220);
    let dim     = egui::Color32::from_rgb(120, 130, 150);
    let green   = egui::Color32::from_rgb(80, 200, 120);
    let orange  = egui::Color32::from_rgb(255, 160, 60);

    egui::Window::new(
        egui::RichText::new(format!("Variable Inspector — Robot {} k={}", resp.robot_id, resp.variable_k))
            .color(heading)
            .size(13.0),
    )
    .default_width(280.0)
    .resizable(true)
    .open(&mut open)
    .show(ctx, |ui| {
        ui.add_space(4.0);

        // Summary row
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("mean").color(dim).monospace().size(11.0));
            ui.label(egui::RichText::new(format!("{:.4}", resp.mean)).monospace().size(11.0));
            ui.add_space(12.0);
            ui.label(egui::RichText::new("var").color(dim).monospace().size(11.0));
            ui.label(egui::RichText::new(format!("{:.4}", resp.variance)).monospace().size(11.0));
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(4.0);

        // Factor summaries
        if resp.factors.is_empty() {
            ui.label(egui::RichText::new("No connected factors").color(dim).italics().size(11.0));
        } else {
            ui.label(
                egui::RichText::new(format!("Connected factors ({})", resp.factors.len()))
                    .color(accent)
                    .strong()
                    .size(11.5),
            );
            ui.add_space(2.0);

            for f in &resp.factors {
                let kind_color = match f.kind.as_str() {
                    "Dynamics"      => egui::Color32::from_rgb(100, 180, 255),
                    "VelocityBound" => egui::Color32::from_rgb(180, 120, 255),
                    "InterRobot"    => orange,
                    _               => green,
                };

                ui.horizontal(|ui| {
                    ui.label(
                        egui::RichText::new(&f.kind)
                            .color(kind_color)
                            .monospace()
                            .size(10.5),
                    );
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new("eta").color(dim).monospace().size(10.0));
                    ui.label(
                        egui::RichText::new(format!("{:.3}", f.msg_eta))
                            .monospace()
                            .size(10.0),
                    );
                    ui.add_space(6.0);
                    ui.label(egui::RichText::new("lam").color(dim).monospace().size(10.0));
                    ui.label(
                        egui::RichText::new(format!("{:.3}", f.msg_lambda))
                            .monospace()
                            .size(10.0),
                    );
                });
            }
        }

        ui.add_space(4.0);
    });

    Ok(())
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use crate::state::InspectFactorInfo;
    use super::*;

    fn make_response(robot_id: u32, k: usize, mean: f32, variance: f32) -> InspectResponse {
        InspectResponse {
            robot_id,
            variable_k: k,
            mean,
            variance,
            factors: vec![
                InspectFactorInfo { kind: "Dynamics".into(), msg_eta: 1.0, msg_lambda: 5.0 },
                InspectFactorInfo { kind: "VelocityBound".into(), msg_eta: 0.5, msg_lambda: 2.0 },
            ],
        }
    }

    #[test]
    fn inspect_response_has_expected_fields() {
        let r = make_response(0, 0, 2.5, 0.01);
        assert_eq!(r.robot_id, 0);
        assert_eq!(r.variable_k, 0);
        assert!((r.mean - 2.5).abs() < 1e-6);
        assert!((r.variance - 0.01).abs() < 1e-6);
        assert_eq!(r.factors.len(), 2);
        assert_eq!(r.factors[0].kind, "Dynamics");
        assert_eq!(r.factors[1].kind, "VelocityBound");
    }

    #[test]
    fn inspect_factor_info_fields() {
        let f = InspectFactorInfo { kind: "InterRobot".into(), msg_eta: 3.14, msg_lambda: 9.81 };
        assert_eq!(f.kind, "InterRobot");
        assert!((f.msg_eta - 3.14).abs() < 1e-5);
        assert!((f.msg_lambda - 9.81).abs() < 1e-5);
    }

    #[test]
    fn inspect_response_with_no_factors() {
        let r = InspectResponse {
            robot_id: 2,
            variable_k: 3,
            mean: 0.0,
            variance: 100.0,
            factors: vec![],
        };
        assert_eq!(r.factors.len(), 0);
        assert!((r.variance - 100.0).abs() < 1e-5);
    }
}
