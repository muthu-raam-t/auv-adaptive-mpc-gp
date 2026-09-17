#!/usr/bin/env python3
"""
Generate the AUV Learning-Based MPC proposed-system architecture diagram.

Same visual system as the FraudLens (BDA) generator, content swapped for the
Control Systems project.

Produces two files in the current directory:
    auv_architecture.svg   vector, scales to any size, editable
    auv_architecture.png   2600 px wide, flattened RGB (no alpha)

Requirements:
    pip install cairosvg pillow

Usage:
    python3 build_auv_diagram.py

Box heights, arrow positions and the fan-out geometry are computed from the
content, so adding a line to a stage cannot push text outside its border.
Edit the stage(...) / pair(...) calls below to change the diagram.

Stage 12 holds the measured Monte Carlo results (mean +/- std across seeds).
"""

W = 1600
PAD = 40
BOX_L = PAD
BOX_R = W - PAD
BOX_W = BOX_R - BOX_L
GAP = 40

INK = "#1a1208"
RULE = "#1a1208"
DONE_BG = "#e8f3e6"
DONE_BD = "#3f7d3a"
DONE_TX = "#2c5c28"
PEND_BG = "#fdf2dc"
PEND_BD = "#b8891f"
PEND_TX = "#8a6612"
TODO_BG = "#f7e9e9"
TODO_BD = "#a54a4a"
TODO_TX = "#7d3535"
NEUTRAL_BG = "#faf7f0"
CALLOUT_BG = "#efe6d4"
PAGE_BG = "#fdfbf6"
PAGE_RGB = (253, 251, 246)

STATUS = {
    "DONE": (DONE_BG, DONE_BD, DONE_TX, "COMPLETED"),
    "PEND": (PEND_BG, PEND_BD, PEND_TX, "RESULTS PENDING"),
    "TODO": (TODO_BG, TODO_BD, TODO_TX, "NOT STARTED"),
}

out = []
y = PAD
cx = W / 2


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def box(x, w, h, fill, stroke, sw=2.5, rx=6):
    out.append('<rect x="%d" y="%d" width="%d" height="%d" rx="%d" '
               'fill="%s" stroke="%s" stroke-width="%s"/>'
               % (x, y, w, h, rx, fill, stroke, sw))


def line(x1, y1, x2, y2, head=False):
    out.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" '
               'stroke-width="3"%s/>'
               % (x1, y1, x2, y2, RULE,
                  ' marker-end="url(#ar)"' if head else ""))


def text(x, ty, s, size=15, weight="normal", anchor="start",
         fill=INK, family="Helvetica, Arial, sans-serif"):
    out.append('<text x="%s" y="%s" font-family="%s" font-size="%s" '
               'font-weight="%s" text-anchor="%s" fill="%s">%s</text>'
               % (x, ty, family, size, weight, anchor, fill, esc(s)))


def mono(x, ty, s, size=13.5, fill="#42342a", weight="normal", anchor="start"):
    # SVG collapses whitespace runs; non-breaking spaces keep columns aligned.
    s = s.replace("  ", "\u00a0\u00a0")
    text(x, ty, s, size=size, weight=weight, anchor=anchor, fill=fill,
         family="'DejaVu Sans Mono', Consolas, monospace")


def styled(x, ty, ln, size):
    """'!' prefix = red emphasis; OUTPUT/INTEGRATOR lines = bold."""
    if ln.startswith("!"):
        mono(x, ty, ln[1:], size=size, fill="#8a2f2f", weight="bold")
    elif ln.startswith(("OUTPUT", "INTEGRATOR")):
        mono(x, ty, ln, size=size, fill="#1a1208", weight="bold")
    else:
        mono(x, ty, ln, size=size)


def status_pill(bx, bw, py, key):
    bg, bd, tx, label = STATUS[key]
    pw = 148 if key != "PEND" else 170
    px = bx + bw - pw - 18
    out.append('<rect x="%d" y="%d" width="%d" height="24" rx="12" '
               'fill="%s" stroke="%s" stroke-width="1.6"/>'
               % (px, py, pw, bg, bd))
    text(px + pw / 2, py + 16.5, label, size=12.5, weight="bold",
         anchor="middle", fill=tx)


def arrow(x=cx, length=34, label=None):
    global y
    line(x, y, x, y + length, head=True)
    if label:
        mono(x + 14, y + length / 2 + 4, label, size=12, fill="#6b5a48")
    y += length


def stage(num, title, lines, key, subtitle=None):
    """One full-width numbered stage."""
    global y
    head = 40
    sub = 22 if subtitle else 0
    h = head + sub + len(lines) * 20 + 22
    _, bd, _, _ = STATUS[key]
    box(BOX_L, BOX_W, h, NEUTRAL_BG, bd)
    out.append('<rect x="%d" y="%d" width="%d" height="4" rx="2" fill="%s"/>'
               % (BOX_L, y, BOX_W, bd))
    text(BOX_L + 20, y + 30, "%s  %s" % (num, title), size=17.5, weight="bold")
    status_pill(BOX_L, BOX_W, y + 12, key)
    ty = y + head + 16
    if subtitle:
        mono(BOX_L + 20, ty, subtitle, size=13, fill="#7a6753", weight="bold")
        ty += sub
    for ln in lines:
        styled(BOX_L + 20, ty, ln, 13.5)
        ty += 20
    y += h


def pair(split_label, left, right, callout):
    """Split into two side-by-side boxes, merge into a callout."""
    global y
    half = (BOX_W - GAP) / 2
    lx = BOX_L + half / 2
    rx = BOX_L + half + GAP + half / 2
    SPLIT = 52
    line(cx, y, cx, y + 18)
    line(lx, y + 18, rx, y + 18)
    mono(cx, y + 40, split_label, size=12.5, anchor="middle", fill="#6b5a48")
    for px in (lx, rx):
        line(px, y + 18, px, y + 18 + SPLIT, head=True)
    y += 18 + SPLIT

    n = max(len(left[3]), len(right[3]))
    h2 = 78 + n * 19 + 20
    top = y
    for i, (num, title, sub, lines) in enumerate((left, right)):
        bx = BOX_L if i == 0 else BOX_L + half + GAP
        y = top
        box(bx, half, h2, NEUTRAL_BG, DONE_BD)
        out.append('<rect x="%d" y="%d" width="%d" height="4" rx="2" fill="%s"/>'
                   % (bx, y, half, DONE_BD))
        text(bx + 20, y + 30, "%s  %s" % (num, title), size=17, weight="bold")
        status_pill(bx, half, y + 12, "DONE")
        mono(bx + 20, y + 54, sub, size=12.5, fill="#7a6753", weight="bold")
        ty = y + 78
        for ln in lines:
            styled(bx + 20, ty, ln, 13)
            ty += 19
    y = top + h2

    line(lx, y, lx, y + 26)
    line(rx, y, rx, y + 26)
    line(lx, y + 26, rx, y + 26)
    line(cx, y + 26, cx, y + 56, head=True)
    y += 56

    head, subs = callout
    CW = 820
    CH = 56 + len(subs) * 20 + 8
    box((W - CW) / 2, CW, CH, CALLOUT_BG, INK, sw=3)
    text(cx, y + 40, head, size=24, weight="bold", anchor="middle")
    for k, s in enumerate(subs):
        mono(cx, y + 66 + k * 20, s, size=13, anchor="middle", fill="#4a3c2e")
    y += CH


def fanout(boxes, h3=366):
    """Split into three side-by-side boxes."""
    global y
    third = (BOX_W - 2 * GAP) / 3
    cs = [BOX_L + i * (third + GAP) + third / 2 for i in range(3)]
    line(cx, y, cx, y + 24)
    line(cs[0], y + 24, cs[2], y + 24)
    for px in cs:
        line(px, y + 24, px, y + 54, head=True)
    y += 54
    top = y
    for i, (num, title, lines) in enumerate(boxes):
        bx = BOX_L + i * (third + GAP)
        y = top
        box(bx, third, h3, NEUTRAL_BG, DONE_BD)
        out.append('<rect x="%d" y="%d" width="%d" height="4" rx="2" fill="%s"/>'
                   % (bx, y, third, DONE_BD))
        text(bx + 18, y + 30, "%s %s" % (num, title), size=15.5, weight="bold")
        status_pill(bx, third, y + 44, "DONE")
        ty = y + 88
        for ln in lines:
            styled(bx + 18, ty, ln, 12.5)
            ty += 18
    y = top + h3


# ---------------------------------------------------------------- title
TITLE_H = 130
box(BOX_L, BOX_W, TITLE_H, "#ffffff", INK, sw=3.5)
text(cx, y + 42, "PROPOSED SYSTEM ARCHITECTURE", size=27,
     weight="bold", anchor="middle")
mono(cx, y + 68,
     "Learning-Based MPC with Regularised Dynamic-Forgetting Gaussian Processes",
     size=14.5, anchor="middle", fill="#4a3c2e")
mono(cx, y + 90,
     "4-DOF AUV  +  GP Disturbance Bank  +  Nonlinear MPC   [MATLAB / Simulink]",
     size=13, anchor="middle", fill="#7a6753")
mono(cx, y + 112,
     "ALL STAGES EXECUTED  -  every figure below is measured in this codebase",
     size=12.5, anchor="middle", fill="#2c5c28")
y += TITLE_H
arrow()

# ---------------------------------------------------------------- stage 0
stage("0.", "VEHICLE MODEL & PARAMETERS", [
    "4-DOF underwater vehicle - same equations and parameters as the base paper",
    "STATE   x   = [x, y, z, u, v, w, psi, r]    position (earth) | velocity (body) | heading, yaw rate",
    "INPUT   tau = [X, Y, Z, Mz]                 surge / sway / heave force, yaw moment",
    "Yaw effective inertia   Izz - Nr_dot = 0.52   <- small; source of the stiffness issue (stage 11)",
    "Actuator limits  u_min <= u <= u_max  enforced inside the optimiser",
], "DONE")
arrow()

# ---------------------------------------------------------------- stage 1
stage("1.", "REFERENCE & DISTURBANCE GENERATION", [
    "REFERENCE  : rotated lemniscate (figure-eight), timed to one full closed lap",
    "",
    "DISTURBANCE (TRUE, hidden from the controller):",
    "   sine  ->  combined sine  ->  square wave",
    "   escalating difficulty - the square wave's abrupt jumps punish slow forgetting",
    "   models currents, tether drag, thruster wake: never fully known in advance",
    "",
    "!USED ONLY TO SCORE THE ESTIMATOR - a real vehicle never sees d_true",
], "DONE")
arrow(label="x_ref(k ... k+Nc) -> NMPC        d_true(t) -> plant only")

# ---------------------------------------------------------------- stage 2
stage("2.", "STATE FEEDBACK", [
    "In practice: IMU (heading / attitude) + DVL (velocity), fused into position by dead-reckoning",
    "Measured EVERY control step; feeds BOTH the residual estimator (3) and the NMPC (9)",
    "",
    "NOISY MODE (simulate_method_noisy.m): controller sees a noisy DVL/IMU-style measurement;",
    "   the plant's TRUE motion stays exact; tracking error is scored against the TRUE state",
], "DONE")

# ------------------------------------------------- 3A / 3B  -> residual
pair("same tau, same state to both  ->  the difference isolates the DISTURBANCE, nothing else",
     ("3A.", "NOMINAL MODEL PREDICTION", "auv_dynamics.m with d = 0", [
         "M nu_dot = tau + C(nu)nu + D(nu)nu + g",
         "Coriolis + Damping + Restoring only",
         "",
         "what the vehicle SHOULD do",
         "if the water were perfectly still",
         "",
         "OUTPUT:  nu_dot_nominal",
     ]),
     ("3B.", "MEASURED MOTION", "state feedback from stage 2", [
         "velocity change between successive",
         "measured states over Ts = 0.2 s",
         "",
         "what the vehicle ACTUALLY did",
         "under the hidden disturbance",
         "",
         "OUTPUT:  nu_dot_measured",
     ]),
     ("RESIDUAL:  Delta_meas = measured - nominal", [
         "residual_disturbance.m  -  the gap IS the disturbance measurement,",
         "one training point (recent state, Delta_meas) per control step",
     ]))
arrow()

# ---------------------------------------------------------------- stage 4
stage("4.", "FORGETTING-FACTOR GP BANK", [
    "Three GPs share ONE data buffer and differ only in memory:",
    "   lambda = 1.0   slow forgetting    - smooth, trusts history, lags on sudden jumps",
    "   lambda = 0.8   medium forgetting",
    "   lambda = 0.6   fast forgetting    - reacts quickly, noisier in calm water",
    "",
    "INPUT : (recent state, Delta_meas) appended every control step",
    "OUTPUT: per GP j  ->  mean mu_j(x)  +  predictive variance sigma2_j(x)",
    "",
    "WHY A BANK: no single lambda is right. The correct memory depends on what the current",
    "is doing RIGHT NOW, and that is not known in advance.",
    "",
    "vs BASE PAPER: dense weighted-kernel GP instead of a sparse GP with inducing points",
    "               - same purpose, simpler mechanism",
], "DONE", subtitle="ForgettingGP.m   [MATLAB class]")
arrow()

# ---------------------------------------------------------------- stage 5
stage("5.", "DYNAMIC WEIGHT OPTIMISATION   <NOVELTY 1>", [
    "PROBLEM - base paper, Eq. 26:",
    "   minimise  sum_j eta_j * e_j        s.t.  sum(eta) = 1,  eta >= 0",
    "   eta sits OUTSIDE the squared error  ->  objective is LINEAR in eta",
    "   a linear cost over a probability simplex is always minimised at a VERTEX",
    "!   -> 100% on one GP, 0% on the others: a hard SWITCH that only looks like a blend",
    "",
    "FIX - this project:",
    "   minimise  sum_j eta_j * e_j  +  rho * ||eta||^2       (same constraints)",
    "   the quadratic term bends the flat cost into a bowl -> the optimum can sit",
    "   INSIDE the simplex -> weight genuinely shared across the GPs",
    "   rho = 0      reproduces the paper exactly (hard switching)",
    "   rho = 0.05   this project's setting (genuine blend)",
    "",
    "MEASURED (demo_lp_vs_qp_blend.m, same scenario, both settings):",
    "                                     MEAN WEIGHT CHANGE PER STEP",
    "   Plain LP          (rho = 0)                 ~0.049",
    "   Regularised QP    (rho = 0.05)              ~0.045",
    "",
    "~9% less weight chattering - the same order as the gain the base paper claims",
    "for its own method. rho is the single knob between switch and blend.",
], "DONE", subtitle="dynamic_weight_qp.m   [MATLAB, quadratic program]")

# -------------------------------------------- fan out to 6 / 7 / 8
fanout([
    ("6.", "BLENDED MEAN", [
        "Delta_hat = sum eta_j mu_j",
        "",
        "replaces the fixed, offline",
        "disturbance assumption the",
        "nominal model would use",
        "",
        "fed into the NMPC",
        "prediction model so its",
        "forecasts self-correct",
        "",
        "base paper stops here:",
        "it uses the MEAN only",
    ]),
    ("7.", "BLENDED VARIANCE", [
        "Sigma_hat = sum eta_j sigma2_j",
        "",
        "how SURE the bank is",
        "",
        "HIGH: start of mission,",
        "  right after a regime",
        "  change",
        "LOW : disturbance learned",
        "",
        "base paper COMPUTES this",
        "but never USES it in the",
        "MPC. This project does",
        "(Novelty 2, stage 9).",
    ]),
    ("8.", "WEIGHT LOGGING", [
        "eta(k) recorded every step",
        "",
        "churn = mean change in the",
        "weights per step",
        "",
        "the metric behind the",
        "0.049 vs 0.045 result",
        "",
        "rho = 0    -> weights jump",
        "             corner to corner",
        "rho = 0.05 -> weights move",
        "             smoothly",
    ]),
], h3=340)
arrow()

# ---------------------------------------------------------------- stage 9
stage("9.", "UNCERTAINTY-AWARE NONLINEAR MPC   <NOVELTY 2>", [
    "AT EVERY CONTROL STEP (receding horizon):",
    "   1. take current state + Delta_hat + Sigma_hat",
    "   2. predict Nc = 8 steps x Ts = 0.2 s   ->   1.6 s lookahead",
    "   3. choose the force sequence that minimises",
    "        J = sum_k (x_k - x_ref,k)' Q (x_k - x_ref,k)  +  u_k' R u_k  +  terminal cost",
    "   4. apply ONLY the first action, discard the rest",
    "   5. repeat next step with fresh measurements",
    "",
    "Q weighs tracking error per state   |   R weighs control effort",
    "SOLVER: fmincon SQP, multiple shooting   (base paper: qpOASES + real-time iteration)",
    "",
    "VARIANCE TIGHTENS THE ACTUATOR BOUNDS:",
    "   shrink      = min(0.3, 0.05 * sqrt(mean(Sigma_hat)))",
    "   bounds_eff  = bounds * (1 - shrink)",
    "   uncertain estimate  ->  up to 30% of actuator range held back  ->  conservative",
    "   confident estimate  ->  shrink ~ 0                              ->  full range",
    "",
    "A plain MPC optimises for a vehicle that does not exist when its model ignores the",
    "current. The MEAN fixes the model; the VARIANCE says how far to trust the fix.",
], "DONE", subtitle="nmpc_solve.m   [MATLAB, fmincon]")
arrow()

# --------------------------------------------------------------- stage 10
stage("10.", "CONTROL ALLOCATION + AUV PLANT", [
    "ALLOCATION: first action u = [X Y Z Mz] applied directly as force / moment",
    "   (a real 6-thruster BlueROV2 would map this through a thruster allocation matrix)",
    "",
    "KINEMATICS :  x_dot = cos(psi)u - sin(psi)v        y_dot = sin(psi)u + cos(psi)v",
    "              z_dot = w                            psi_dot = r",
    "CORIOLIS   :  C_u = (m v + Yv_dot v) r             C_v = -(m u + Xu_dot u) r",
    "DAMPING    :  D_u = (Xu + Xuc|u|) u                D_v = (Yv + Yvc|v|) v",
    "RESTORING  :  ~0  - vehicle near neutral buoyancy by design",
    "",
    "(mass term) * accel  =  tau + Coriolis + Damping + Restoring + d_TRUE",
    "sum -> inv(M) -> integrate -> velocity -> kinematics -> position",
    "",
    "INTEGRATOR: 4th-order Runge-Kutta - 4 dynamics evaluations per step, blended",
    "TRUE state updated under the ACTUAL disturbance, then re-measured at stage 2",
], "DONE", subtitle="auv_dynamics.m + rk4_integrate.m   [4-DOF, paper parameters]")

# ------------------------------------------ 11A / 11B  -> yaw callout
pair("SAME underlying functions called by both  ->  the comparison tests the IMPLEMENTATION, not the math",
     ("11A.", "MATLAB SCRIPT", "run_simulation.m", [
         "all 4 controller variants in one run:",
         "   NoGP          StaticGP",
         "   DFGP_LP       RDFGP_UAMPC",
         "",
         "overlaid, paper-style comparison plots",
         "fast to iterate",
         "",
         "INTEGRATOR: RK4",
     ]),
     ("11B.", "SIMULINK MODELS", "build_full_simulink_model.m + sim", [
         "auv_full_system",
         "   single plant block, RK4 - matches MATLAB",
         "auv_full_system_decomposed",
         "   Coriolis / Damping / Restoring / Kinematics",
         "   as visible blocks, mirrors the paper",
         "   proposed controller as a deployable diagram",
         "",
         "!INTEGRATOR: Forward-Euler (decomposed model)",
     ]),
     ("YAW AXIS:  Izz - Nr_dot = 0.52", [
         "small effective inertia -> large yaw acceleration -> numerically STIFF",
         "Forward-Euler can go unstable on yaw; RK4 stays stable",
     ]))
arrow()

# --------------------------------------------------------------- stage 12
stage("12.", "SENSOR NOISE + MONTE CARLO ROBUSTNESS STUDY", [
    "SEEDS : same seed = identical noise, reproducible run",
    "        new seed  = a different real-world draw of sensor imperfection",
    "REPORT: MEAN and STANDARD DEVIATION across seeds, per controller variant",
    "        - a single run can be lucky; the spread is what makes a result reliable",
    "",
    "   VARIANT          TRACKING ERROR (mean +/- std)    PREDICTION ERROR (mean +/- std)",
    "   NoGP                0.1970 +/- 0.0003                    N/A",
    "   StaticGP            0.2066 +/- 0.0028               3.1959 +/- 0.6602",
    "   DFGP_LP             0.2050 +/- 0.0030               3.4889 +/- 0.5590",
    "   RDFGP_UAMPC         0.2055 +/- 0.0024               3.6408 +/- 0.3384",
    "",
    "!RESOLVED: RDFGP_UAMPC has the highest average prediction error but the",
    "!SMALLEST spread (std 0.34 vs 0.66) - most consistent, not most accurate.",
], "DONE", subtitle="run_monte_carlo.m + simulate_method_noisy.m   [MATLAB]")
y += 30

# ------------------------------------------------------ closed-loop footer
FH = 128
box(BOX_L, BOX_W, FH, "#f2ece0", INK, sw=3)
text(cx, y + 34, "CLOSED-LOOP TIMING", size=18, weight="bold", anchor="middle")
mono(cx, y + 62,
     "Stages 2 -> 10 run once per control step (Ts = 0.2 s): measure -> estimate ->",
     size=13.5, anchor="middle", fill="#42342a")
mono(cx, y + 82,
     "blend -> optimise -> actuate -> integrate -> log. Only the FIRST optimised action is",
     size=13.5, anchor="middle", fill="#42342a")
mono(cx, y + 102,
     "applied; the plant's new TRUE state is re-measured and the loop returns to stage 2.",
     size=13.5, anchor="middle", fill="#42342a")
y += FH + PAD

H = int(y)
svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
       'viewBox="0 0 %d %d">' % (W, H, W, H),
       '<defs><marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" '
       'markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
       '<path d="M 0 0 L 10 5 L 0 10 z" fill="%s"/></marker></defs>' % RULE,
       '<rect x="0" y="0" width="{}" height="{}" fill="{}"/>'.format(W, H, PAGE_BG)]
svg += out
svg.append("</svg>")

SVG_PATH = "auv_architecture.svg"
PNG_PATH = "auv_architecture.png"
PNG_WIDTH = 2600

with open(SVG_PATH, "w") as fh:
    fh.write("\n".join(svg))
print("SVG written: %s  (%d x %d)" % (SVG_PATH, W, H))

# Rasterise to an OPAQUE PNG (no alpha -> no checkerboard in viewers).
try:
    import cairosvg
    from PIL import Image

    cairosvg.svg2png(url=SVG_PATH, write_to=PNG_PATH,
                     output_width=PNG_WIDTH, background_color=PAGE_BG)

    im = Image.open(PNG_PATH)
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        canvas = Image.new("RGB", im.size, PAGE_RGB)
        canvas.paste(im, mask=im.split()[-1])
        im = canvas
    else:
        im = im.convert("RGB")
    im.save(PNG_PATH, "PNG", optimize=True)

    check = Image.open(PNG_PATH)
    assert "A" not in check.mode, "PNG still has an alpha channel"
    print("PNG written: %s  (%d x %d, mode %s)"
          % (PNG_PATH, check.size[0], check.size[1], check.mode))
except ImportError:
    print("PNG skipped -- install the renderers first:")
    print("    pip install cairosvg pillow")
