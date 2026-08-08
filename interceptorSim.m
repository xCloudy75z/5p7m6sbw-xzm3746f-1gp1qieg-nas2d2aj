function out = interceptorSim(overrides, verbose)
  % interceptorSim  SINGLE-FILE bundle of the interceptor engagement simulator.
  % GENERATED from src2/ by build_bundle.sh -- DO NOT EDIT HERE; edit the src2/ modules and re-run
  % the generator (the 214-test suite runs against src2/, not this bundle).
  %
  % Run (from the folder holding this file, Predator.STL and data/):
  %   interceptorSim()                                  % default synthetic run
  %   interceptorSim(struct('mesh_source','stl','stl_file','Predator.STL', ...
  %                         'source','file','file','data/fragments-sample.csv'))   % real drone + real fragments
  if nargin<1, overrides=[]; end
  if nargin<2 || isempty(verbose), verbose=true; end
  out = main(overrides, verbose);
end

% ==================== main.m ====================
function out = main(overrides, verbose)
  % main  Thin entry: config -> fragments -> mesh -> propagate -> coverage -> budget -> timing -> summary.
  % Returns a results struct; prints the summary unless verbose=false. Default is drag-off (fast); toggle
  % p.phys.drag for the defensible with-drag KE. loadFragments is a later seam (only reached when source='file').
  if nargin<2 || isempty(verbose), verbose=true; end
  if nargin>=1 && ~isempty(overrides), p=config(overrides); else, p=config(); end
  if strcmp(p.uav.mesh_source,'stl')
    mesh = loadSTLmesh(p.uav.stl_file, p.uav);
  else
    mesh = makeUAVmesh(p.uav);
  end
  if strcmp(p.frag.source,'synthetic')
    V = syntheticFragments(p);
    res = arrayfun(@(i) propagateFragments(V(i,:), p, mesh), (1:size(V,1))');
  else
    frags = loadFragments(p.frag.file);
    mf = mapFragments(frags, p, mesh);                 % orient into engine frame + place burst (real-data geom)
    res = arrayfun(@(i) propagateFragments(mf.Vej(i,:), p, mesh, mf.mass_kg(i), mf.burst), (1:mf.n)');
  end
  cov = coverageLethality(res, mesh);
  db  = delayBudget(p);
  tm  = timingModes(res, p);
  out = struct('p',p,'results',res,'coverage',cov,'budget',db,'timing',tm);
  out.summary = summaryReport(p, res, cov, db, tm);
  if verbose, fprintf('%s\n', out.summary); end
end

% ==================== config.m ====================
function p = config(overrides)
  % config  Validated parameter struct for the fragment-propagation core (dual-safe).
  % Units: SI internally (m, m/s, s, rad, kg). Degrees/mm/ms only as input, converted here.
  % ---- USER KNOBS ----
  p.interceptor.speed   = 138.9;          % m/s (platform carry)
  p.target.speed        = 87.5;           % m/s (UAV, head-on -x)
  p.datum.radial_offset = 5.0;            % m across-track (y)
  p.burst = struct('miss_m',5.0,'clock_deg',0);  % skin-based miss + clock position (0=12 o'clock=+z). Real-data geom.
  % UAV: small fast-UAV / loitering-munition class (session-2 decision; swappable seam).
  p.uav = struct('fuselage_len',2.5,'fuselage_dia',0.35,'wingspan',3.0,'wing_chord',0.35,'tail_h',0.6,'attitude_deg',[0 0 0]);
  p.uav.mesh_source = 'proc';             % 'proc' (procedural placeholder) | 'stl' (real mesh)
  p.uav.stl_file    = '';                 % path to a binary STL when mesh_source='stl'
  p.frag.source   = 'synthetic';
  p.frag.file     = '';                   % path to a real-fragment CSV when source='file'
  p.frag.count    = 200;
  p.frag.speed_range = [800 1200];        % m/s
  p.frag.eject_range_deg = [-15 15];
  p.frag.z_spread_deg = 10;
  p.frag.seed     = 42;
  p.frag.material = 'steel';              % steel|tungsten
  p.frag.shape    = 'cube';               % cube|sphere
  p.frag.size_mm  = 5;
  p.phys.drag = false; p.phys.gravity = false; p.phys.integrator = 'rk4'; p.phys.dt_ms = 0.05; % rk4 default converges at a ~10x coarser/faster step than euler (break-build perf fix; rk4 is spec's named upgrade)
  p.timing.mode = 'both'; p.heatmap = false;
  % ---- FIXED ----
  p.delays.sensing1 = [500e-6 1000e-6]; p.delays.sensing2 = [200e-6 200e-6];
  p.rho_air = 1.225;
  p.phys.Tmax = 0.2;   % s. Integration horizon cap (ample for a ~6 m path at >=700 m/s).
  % ---- OVERRIDES (tests) ----
  if nargin>=1 && isstruct(overrides)
    fn = fieldnames(overrides);
    for i=1:numel(fn)
      switch fn{i}
        case 'radial_offset', p.datum.radial_offset = overrides.radial_offset;
        case 'material', p.frag.material = overrides.material;
        case 'size_mm', p.frag.size_mm = overrides.size_mm;
        case 'shape', p.frag.shape = overrides.shape;
        case 'count', p.frag.count = overrides.count;
        case 'seed', p.frag.seed = overrides.seed;
        case 'integrator', p.phys.integrator = overrides.integrator;
        case 'source', p.frag.source = overrides.source;
        case 'file', p.frag.file = overrides.file;
        case 'clock_deg', p.burst.clock_deg = overrides.clock_deg;
        case 'miss_m', p.burst.miss_m = overrides.miss_m;
        case 'dt_ms', p.phys.dt_ms = overrides.dt_ms;
        case 'drag', p.phys.drag = logical(overrides.drag);
        case 'gravity', p.phys.gravity = logical(overrides.gravity);
        case 'mesh_source', p.uav.mesh_source = overrides.mesh_source;
        case 'stl_file',    p.uav.stl_file    = overrides.stl_file;
        otherwise, error('config: unknown override field "%s"', fn{i});
      end
    end
  end
  % ---- VALIDATION ----
  if p.datum.radial_offset<=0, error('config: radial_offset must be > 0'); end
  if p.interceptor.speed<=0 || p.target.speed<=0, error('config: speeds must be > 0'); end
  if p.frag.count<1 || p.frag.count~=fix(p.frag.count), error('config: count must be a positive integer'); end
  if p.frag.size_mm<=0, error('config: size_mm must be > 0'); end
  if p.frag.speed_range(1)<=0 || p.frag.speed_range(1)>p.frag.speed_range(2), error('config: bad speed_range'); end
  if ~any(strcmp(p.frag.material,{'steel','tungsten'})), error('config: material must be steel|tungsten'); end
  if ~any(strcmp(p.frag.shape,{'cube','sphere'})), error('config: shape must be cube|sphere'); end
  if ~any(strcmp(p.phys.integrator,{'euler','rk4'})), error('config: integrator must be euler|rk4'); end
  if ~any(strcmp(p.frag.source,{'synthetic','file'})), error('config: source must be synthetic|file'); end
  if ~isscalar(p.burst.clock_deg) || ~isnumeric(p.burst.clock_deg) || ~isfinite(p.burst.clock_deg), error('config: burst.clock_deg must be a finite scalar'); end
  if ~isscalar(p.burst.miss_m) || ~isnumeric(p.burst.miss_m) || ~isfinite(p.burst.miss_m) || p.burst.miss_m<=0, error('config: burst.miss_m must be a positive finite scalar'); end
  if strcmp(p.frag.source,'file')
    if isempty(p.frag.file), error('config: source=file requires p.frag.file (a path)'); end
    fid_ = fopen(p.frag.file,'r');
    if fid_ < 0, error('config: source=file but cannot open "%s"', p.frag.file); end
    fclose(fid_);
  end
  if ~any(strcmp(p.uav.mesh_source,{'proc','stl'})), error('config: uav.mesh_source must be proc|stl'); end
  if strcmp(p.uav.mesh_source,'stl')
    if isempty(p.uav.stl_file), error('config: mesh_source=stl requires uav.stl_file (a path)'); end
    fid_s = fopen(p.uav.stl_file,'r');
    if fid_s < 0, error('config: mesh_source=stl but cannot open "%s"', p.uav.stl_file); end
    fclose(fid_s);
  end
  if p.phys.dt_ms<=0, error('config: dt_ms must be > 0'); end
  if ~isscalar(p.phys.drag)    || ~(islogical(p.phys.drag)||isnumeric(p.phys.drag)),    error('config: drag must be a scalar logical'); end
  if ~isscalar(p.phys.gravity) || ~(islogical(p.phys.gravity)||isnumeric(p.phys.gravity)), error('config: gravity must be a scalar logical'); end
  % ---- DERIVED: fragment mass (kg) + area (m^2) + drag Cd + k ----
  switch p.frag.material, case 'steel', dens=7.85; case 'tungsten', dens=19.3; end  % mg/mm^3
  switch p.frag.shape
    case 'cube',   vol_mm3=p.frag.size_mm^3;                 area=((p.frag.size_mm/1000)^2)*6/4;   Cd=1.14; % ITOP 4-2-813 cube; tumbling-avg area (Sperrazza)
    case 'sphere', vol_mm3=(4/3)*pi*(p.frag.size_mm/2)^3;    area=pi*((p.frag.size_mm/1000)/2)^2;  Cd=0.93; % ITOP 4-2-813 ball
  end
  p.frag.mass_kg = dens * vol_mm3 * 1e-6;    % mg->kg is 1e-6 (break-spec-2 #1)
  p.frag.area_m2 = area; p.frag.Cd = Cd;
  p.frag.k_per_m = 0.5 * Cd * p.rho_air * area / p.frag.mass_kg;  % quadratic-drag decay const
end

% ==================== coverageLethality.m ====================
function cov = coverageLethality(res, mesh)
  % coverageLethality  Aggregate per-fragment results: hit count/by-part, KE, coverage, density.
  hit=[res.hit]; noncr=[res.noncrossing];
  cov.n_hits=sum(hit); cov.n_noncrossing=sum(noncr); cov.n_total=numel(res);
  cov.hit_fraction=cov.n_hits/max(1,cov.n_total);
  parts={res(hit).part};
  cov.n_fuselage=sum(strcmp(parts,'fuselage')); cov.n_wing=sum(strcmp(parts,'wing')); cov.n_tail=sum(strcmp(parts,'tail'));
  KE=[res(hit).KE_J]; cov.KE_total_J=sum(KE);
  if cov.n_hits>0, cov.KE_per_hit_J=mean(KE); else, cov.KE_per_hit_J=0; end
  % per-face triangle areas
  nF=size(mesh.F,1); face_area=zeros(nF,1);
  for i=1:nF, a=mesh.V(mesh.F(i,1),:); b=mesh.V(mesh.F(i,2),:); c=mesh.V(mesh.F(i,3),:); face_area(i)=0.5*norm(cross(b-a,c-a)); end
  cov.mesh_area_m2=sum(face_area);
  % struck faces (unique indices that were hit) -> coverage = struck/total, density = hits/struck (spec 10f)
  struck=unique([res(hit).face_idx]); struck=struck(struck>0);
  cov.struck_area_m2=sum(face_area(struck));
  cov.coverage=cov.struck_area_m2/max(cov.mesh_area_m2,eps);
  cov.density_hits_per_m2 = cov.n_hits / max(cov.struck_area_m2,eps);
end

% ==================== delayBudget.m ====================
function db = delayBudget(p)
  % delayBudget  Closing speed + electronics delay window + delay budget (nominal frag flight).
  db.closing_speed = p.target.speed + p.interceptor.speed;   % head-on additive
  s1=p.delays.sensing1; s2=p.delays.sensing2;
  db.elec_delay_min_s = s1(1)+s2(1); db.elec_delay_max_s = s1(2)+s2(2);
  % representative fragment flight time t3 = offset / nominal frag speed (perpendicular)
  fnom = mean(p.frag.speed_range);
  t3 = p.datum.radial_offset / fnom;
  terms=[s1(1), s2(1), t3];             % [sensing1 sensing2 fragment], min-electronics window
  db.total_window_s = sum(terms);
  db.budget_pct = 100*terms/sum(terms);  % sums to 100 by construction
  db.t3_s = t3;
end

% ==================== facesEquivOK.m ====================
function ok = facesEquivOK(m,i,j)
  % facesEquivOK  Equivalence-test tolerance: are struck faces i,j acceptably "the same"?
  % True if identical, OR if they share a vertex position (a genuine geometric tie — a segment through a
  % shared edge/vertex may pick either adjacent face; only the part label differs). This test is INDEPENDENT
  % of t, so a wrong, non-adjacent face is NOT excused. (break-plan BLOCKER fix, 2026-08-08.)
  if i==j, ok=true; return; end
  if i<1 || j<1, ok=false; return; end
  Vi=m.V(m.F(i,:),:); Vj=m.V(m.F(j,:),:); ok=false;
  for a=1:3
    for b=1:3
      if norm(Vi(a,:)-Vj(b,:))<1e-9, ok=true; return; end
    end
  end
end

% ==================== fragAccel.m ====================
function a = fragAccel(vrel, p)
  % fragAccel  Acceleration (m/s^2) on a fragment in the target rest frame.
  % Quadratic drag acts on AIRSPEED v_air = v_rel - (Vt,0,0) (still air is at rest in the ground
  % frame), NOT v_rel (spec section 7, break-spec #2). a_drag = -k|v_air| v_air with
  % k = p.frag.k_per_m (= 1/2 Cd rho A / m). Gravity is (0,0,-9.81). Both toggled by p.phys.
  a = [0 0 0];
  if p.phys.drag
    vair = vrel - [p.target.speed 0 0];
    a = a - p.frag.k_per_m * norm(vair) * vair;
  end
  if p.phys.gravity
    a = a + [0 0 -9.81];
  end
end

% ==================== integrateFragment.m ====================
function r = integrateFragment(Vfrag, p, mesh, mass_kg)
  % integrateFragment  Propagate ONE fragment under drag/gravity in the target rest frame, stepping
  % (Euler/RK4) with per-step continuous collision detection against the UAV mesh. Same result struct
  % as propagateFragments' straight-line branch. KE uses |v_rel| interpolated to the intersection.
  % NOTE: if neither a hit nor the y>yStop far-bound is reached within Tmax, returns hit=false,
  % noncrossing=false (an ordinary miss) — Tmax is a runaway guard, not a normal exit for default inputs.
  if nargin < 4 || isempty(mass_kg), mass_kg = p.frag.mass_kg; end
  Vi=p.interceptor.speed; Vt=p.target.speed; d=p.datum.radial_offset;
  vrel=[Vi+Vfrag(1)+Vt, Vfrag(2), Vfrag(3)];
  r=struct('hit',false,'noncrossing',false,'timeout',false,'part','','face_idx',0,'t_s',NaN,'P',[NaN NaN NaN],'KE_J',NaN,'miss_x',NaN,'miss_z',NaN);
  if vrel(2)<=0, r.noncrossing=true; return; end
  tc=d/vrel(2); r.miss_x=vrel(1)*tc; r.miss_z=vrel(3)*tc;   % diagnostic plane point (intuition only)
  dt=p.phys.dt_ms*1e-3; Tmax=p.phys.Tmax;                  % horizon cap (s); ample for a ~6 m path
  pos=[0 -d 0]; t=0; yStop=mesh.bounds.y_max + 0.1;
  while t < Tmax
    [pos2, vrel2]=stepState(pos, vrel, dt, p);
    [hit,tfrac,P,part,face_idx]=meshHitCull(pos, pos2, mesh);
    if hit
      vimp = vrel + (vrel2-vrel)*tfrac;                    % velocity at the intersection
      r.hit=true; r.part=part; r.face_idx=face_idx; r.P=P; r.t_s=t + tfrac*dt;
      r.KE_J=0.5*mass_kg*sum(vimp.^2);
      return;
    end
    pos=pos2; vrel=vrel2; t=t+dt;
    if pos(2) > yStop, return; end                         % passed the mesh far bound -> miss
  end
  r.timeout=true;   % reached Tmax without a hit or far-bound crossing (runaway guard; distinct from a normal miss)
end

% ==================== lcgUniform.m ====================
function u = lcgUniform(n, seed)
  % lcgUniform  Self-contained deterministic uniform generator on [0,1), Nx1.
  % Numerical-Recipes LCG (a,c,m=2^32). Chosen because a*x+c stays < 2^53, so every
  % step is EXACT in IEEE double -> byte-identical output in Octave AND MATLAB, with
  % no dependence on (and no mutation of) the global rand stream.
  a=1664525; c=1013904223; m=4294967296;   % 2^32
  x=mod(seed, m); u=zeros(n,1);
  for i=1:n
    x=mod(a*x + c, m);
    u(i)=x/m;
  end
end

% ==================== loadFragments.m ====================
function frags = loadFragments(file, p)
  % loadFragments  Read the mentor's 18-column fragment CSV -> SI fragment struct (WARHEAD frame).
  % Returns frags.{V (Nx3 m/s = momentum/mass_mg), mass_kg (Nx1), speed (Nx1, =norm(V)),
  %                origin_m (Nx3), n, frame='warhead'}.
  % Dual-safe (Octave + MATLAB), headless, no toolboxes. LOADER-FIRST: no aiming / placement /
  % intercept-frame remap / closing-velocity (those are the next "mapping" step). See
  % docs/superpowers/specs/2026-08-06-loadfragments-real-data-design.md.
  % Columns (1-indexed): 3=Mass(mg), 7-9=Origin(mm), 13=AvgSpeed(m/s), 14-16=Momentum(mg*mm/ms).
  % mm/ms == m/s numerically.
  CONSISTENCY_TOL = 0.02;                         % norm(momentum)/mass must match col-13 speed to 2%
  if nargin < 2, p = []; end                      % accepted for call-site compatibility; unused here
  fid = fopen(file, 'r');
  if fid < 0, error('loadFragments: cannot open file "%s"', file); end
  closer = onCleanup(@() fclose(fid));            % dual-safe guaranteed close
  V = zeros(0,3); mass_kg = zeros(0,1); origin_m = zeros(0,3);
  ntot = 0; ndrop = 0; lineno = 0; seenData = false;
  while true
    line = fgetl(fid);
    if ~ischar(line), break; end                 % EOF (fgetl -> -1)
    lineno = lineno + 1;
    if lineno == 1                               % strip a leading UTF-8 BOM (1-char U+FEFF or 3 raw bytes)
      if numel(line) >= 1 && double(line(1)) == 65279
        line = line(2:end);
      elseif numel(line) >= 3 && double(line(1)) == 239 && double(line(2)) == 187 && double(line(3)) == 191
        line = line(4:end);
      end
    end
    if ~isempty(line) && double(line(end)) == 13, line = line(1:end-1); end   % strip trailing CR
    if isempty(strtrim(line)), continue; end     % blank line
    parts = regexp(line, ',', 'split');          % dual-safe, does NOT collapse empty fields (break-build P2 #1)
    if numel(parts) == 19 && isempty(strtrim(parts{19})), parts = parts(1:18); end  % benign trailing comma
    if ~seenData && isnan(str2double(parts{1}))  % header / comment line: first field non-numeric
      continue;
    end
    seenData = true;
    ntot = ntot + 1;
    if numel(parts) ~= 18                        % malformed row (short, or extra non-empty field) -> drop
      ndrop = ndrop + 1; continue;
    end
    mass_mg = str2double(parts{3});
    mx = str2double(parts{14}); my = str2double(parts{15}); mz = str2double(parts{16});
    ox = str2double(parts{7});  oy = str2double(parts{8});  oz = str2double(parts{9});
    fspeed = str2double(parts{13});              % Average-Speed column (redundant; used only to validate)
    % Required fields must be finite; mass strictly positive. Origin IS validated (it feeds the next
    % step's burst placement) so a NaN origin cannot slip through silently (break-build P1 #3 / P2 #1).
    if ~isfinite(mass_mg) || mass_mg <= 0 || ~isfinite(mx) || ~isfinite(my) || ~isfinite(mz) ...
        || ~isfinite(ox) || ~isfinite(oy) || ~isfinite(oz)
      ndrop = ndrop + 1; continue;
    end
    v = [mx my mz] / mass_mg;                     % mg*mm/ms / mg = mm/ms = m/s
    % Internal consistency guard (break-build P2 #2,#3): the file carries a redundant Average-Speed
    % column. If norm(v) disagrees with it by > CONSISTENCY_TOL, the row is mis-aligned (locale decimal
    % comma, wrong delimiter, shifted columns, or a numeric header/units row) -> drop it. Skipped when
    % the speed column is absent/zero (synthetic fixtures use 0), so it never false-drops clean synthetic data.
    if isfinite(fspeed) && fspeed > 0 && abs(norm(v) - fspeed)/fspeed > CONSISTENCY_TOL
      ndrop = ndrop + 1; continue;
    end
    V(end+1,:)        = v;                         %#ok<AGROW>
    mass_kg(end+1,1)  = mass_mg * 1e-6;           %#ok<AGROW>  mg -> kg
    origin_m(end+1,:) = [ox oy oz] * 1e-3;        %#ok<AGROW>  mm -> m
  end
  if ntot == 0, error('loadFragments: no fragment rows found in "%s"', file); end
  % Loud guard: if a large fraction of rows failed on a non-trivial file, the file format is wrong
  % (delimiter / decimal separator / column alignment) rather than a few stray bad rows (break-build P2 #2).
  if ntot >= 10 && ndrop > 0.2 * ntot
    error(['loadFragments: %d of %d rows in "%s" failed parsing/consistency - likely a delimiter, ' ...
           'decimal-separator (locale), or column-alignment problem in the file'], ndrop, ntot, file);
  end
  if isempty(V), error('loadFragments: no valid fragment rows after filtering "%s"', file); end
  if ndrop > 0
    warning('loadFragments:droppedRows', ...
      ['loadFragments: dropped %d of %d data rows (malformed field count, non-finite/negative mass, ' ...
       'momentum or origin, or momentum inconsistent with the Average-Speed column)'], ndrop, ntot);
  end
  frags.V = V;
  frags.mass_kg = mass_kg;
  frags.speed = sqrt(sum(V.^2, 2));
  frags.origin_m = origin_m;
  frags.n = size(V,1);
  frags.frame = 'warhead';
end

% ==================== loadSTLmesh.m ====================
function m = loadSTLmesh(stl_path, opts)
  % loadSTLmesh  Dual-safe binary-STL loader -> engine-frame, metres, centred, degenerate-dropped,
  % part-tagged {V,F,part,bounds,triMin,triMax} matching makeUAVmesh. No toolboxes. See spec 2026-08-08.
  if nargin<2 || ~isstruct(opts), opts=struct(); end
  scale     = optget(opts,'scale',1e-3);                 % mm -> m
  area_eps  = optget(opts,'area_eps',1e-12);
  axis_map  = optget(opts,'axis_map',[0 0 -1;-1 0 0;0 1 0]); % nose->-x, tail->+x: native nose is +length, needs 180deg about z (det +1). ex=-nz, ey=-nx, ez=ny
  wing_y_frac = optget(opts,'wing_y_frac',0.18);
  wing_z_frac = optget(opts,'wing_z_frac',0.5);
  tail_x_frac = optget(opts,'tail_x_frac',0.55);
  tail_z_frac = optget(opts,'tail_z_frac',0.35);
  if abs(det(axis_map)-1) > 1e-9, error('loadSTLmesh: axis_map must be a proper rotation (det=+1)'); end
  info = dir(stl_path);
  if isempty(info), error('loadSTLmesh: file not found: %s', stl_path); end
  f = fopen(stl_path,'r'); if f<0, error('loadSTLmesh: cannot open %s', stl_path); end
  fread(f,80,'uint8=>uint8'); n = fread(f,1,'uint32'); n=double(n);
  expected = 84 + 50*n;
  if info.bytes ~= expected
    fclose(f); error('loadSTLmesh: size %d != expected %d (not binary STL / corrupt)', info.bytes, expected);
  end
  V=zeros(n*3,3); F=zeros(n,3);
  for i=1:n
    fread(f,3,'float32'); tri=fread(f,9,'float32'); base=(i-1)*3;
    V(base+1,:)=tri(1:3)'; V(base+2,:)=tri(4:6)'; V(base+3,:)=tri(7:9)'; F(i,:)=[base+1 base+2 base+3];
    fread(f,1,'uint16');
  end
  fclose(f);
  V = V*scale;                       % to metres
  V = (axis_map*V')';                % into engine frame
  % Drop degenerate faces FIRST, BEFORE centring/bounds, so a corrupt/orphan degenerate vertex (a common
  % STL export artifact) can never poison the centroid or bounding box (break-build correctness #1).
  A=V(F(:,1),:); B=V(F(:,2),:); C=V(F(:,3),:);
  cr=cross(B-A,C-A,2); area=0.5*sqrt(sum(cr.^2,2));
  keep = area>=area_eps; ndrop=sum(~keep);
  if ndrop>0, warning('loadSTLmesh:degenerate','loadSTLmesh: dropped %d degenerate faces (area<%.1e)',ndrop,area_eps); end
  F = F(keep,:);
  % Drop now-orphaned vertices (of dropped faces) and remap F, so centring/bounds see only real geometry
  % and m.V carries no orphan rows.
  ref = unique(F(:));
  remap = zeros(size(V,1),1); remap(ref) = 1:numel(ref);
  V = V(ref,:); F = remap(F);
  V = V - mean(V,1);                 % centre at centroid (all remaining verts are referenced)
  m.V=V; m.F=F;
  m.bounds = struct('x_min',min(V(:,1)),'x_max',max(V(:,1)),'y_min',min(V(:,2)),'y_max',max(V(:,2)),'z_min',min(V(:,3)),'z_max',max(V(:,3)));
  % per-triangle AABBs + centroids + areas (kept faces)
  V0=V(F(:,1),:); V1=V(F(:,2),:); V2=V(F(:,3),:);
  m.triMin=min(min(V0,V1),V2); m.triMax=max(max(V0,V1),V2);
  Cc=(V0+V1+V2)/3;
  fcr=cross(V1-V0,V2-V0,2); farea=0.5*sqrt(sum(fcr.^2,2));   % kept-face areas (for the up/down orientation signal)
  % part tags by face centroid, fractions of half-extents (spec §4). tail assigned last => tail precedence.
  half_span=(m.bounds.y_max-m.bounds.y_min)/2;
  half_len =(m.bounds.x_max-m.bounds.x_min)/2;
  half_h   =(m.bounds.z_max-m.bounds.z_min)/2;
  m.part = repmat({'fuselage'}, size(F,1), 1);
  isWing = (abs(Cc(:,2)) > wing_y_frac*half_span) & (abs(Cc(:,3)) < wing_z_frac*half_h);
  isTail = (Cc(:,1) > tail_x_frac*half_len)       & (Cc(:,3) > tail_z_frac*half_h);
  m.part(isWing) = {'wing'};
  m.part(isTail) = {'tail'};
  % ---- ORIENTATION guards (warn here; runTests asserts them hard on the real mesh) ----
  % These + the det(axis_map)=+1 assert cover ALL proper-rotation inversions: fore/aft (nose), up/down (z),
  % and left/right (a pure y-flip is det=-1, already blocked; a y+z flip is caught by the up/down check).
  x=Cc(:,1); L=max(x)-min(x);
  fwd = x < min(x)+L/3; aft = x > max(x)-L/3;
  if mean(abs(Cc(fwd,2))+abs(Cc(fwd,3))) >= mean(abs(Cc(aft,2))+abs(Cc(aft,3)))
    warning('loadSTLmesh:orientation','loadSTLmesh: forward cross-section not slimmer than aft; check axis_map (nose should be -x)');
  end
  % up/down: on the real airframe the aft empennage skews VENTRAL (below the wing plane), so the area-weighted
  % aft mean-z is negative. A positive value means the mesh is inverted (a y+z flip that keeps det=+1).
  aftZ = sum(Cc(aft,3).*farea(aft))/sum(farea(aft));
  if aftZ > 0
    warning('loadSTLmesh:orientation','loadSTLmesh: aft section skews dorsal (aft mean-z=%+.3f>0); mesh may be inverted - check axis_map z.', aftZ);
  end
end

function v = optget(s,name,default)
  % NOTE: a present, non-[] value is used verbatim -- so an explicit numeric 0 (e.g. scale=0) is taken as a
  % real value, NOT defaulted. No current option treats 0 as meaningful, so this is safe; callers adding a
  % numeric option where 0 must fall back to the default must not rely on optget for that.
  if isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
end

% ==================== makeUAVmesh.m ====================
function m = makeUAVmesh(u)
  % makeUAVmesh  Procedural UAV triangle mesh. Returns V (Kx3), F (Mx3 idx), part (Mx1 cellstr),
  % bounds. Built level (nose -x, wings +/-y, tail +z), then rotated by attitude_deg (Rz*Ry*Rx).
  hl=u.fuselage_len/2; hd=u.fuselage_dia/2; hs=u.wingspan/2; hc=u.wing_chord/2;
  V=[]; F=[]; part={};
  function addQuad(a,b,c,d,tag)
    base=size(V,1); V=[V;a;b;c;d]; F=[F; base+[1 2 3]; base+[1 3 4]]; part=[part; tag; tag];
  end
  % fuselage: a thin box centred at origin (length x, dia y and z)
  addQuad([-hl -hd 0],[hl -hd 0],[hl hd 0],[-hl hd 0],'fuselage');   % top-ish (z=0 slab, kept simple)
  addQuad([-hl 0 -hd],[hl 0 -hd],[hl 0 hd],[-hl 0 hd],'fuselage');   % side slab (x-z)
  % wing: horizontal plate at z=0, spanning +/-hs in y, chord +/-hc in x
  addQuad([-hc -hs 0],[hc -hs 0],[hc hs 0],[-hc hs 0],'wing');
  % tail fin: vertical plate at rear (x ~ +hl), z 0..tail_h, small y
  xt=hl-hc;
  addQuad([xt -0.02 0],[xt+2*hc -0.02 0],[xt+2*hc -0.02 u.tail_h],[xt -0.02 u.tail_h],'tail');
  % attitude rotation R=Rz*Ry*Rx (roll->pitch->yaw)
  r=u.attitude_deg(1)*pi/180; pit=u.attitude_deg(2)*pi/180; y=u.attitude_deg(3)*pi/180;
  Rx=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
  Ry=[cos(pit) 0 sin(pit);0 1 0;-sin(pit) 0 cos(pit)];
  Rz=[cos(y) -sin(y) 0;sin(y) cos(y) 0;0 0 1];
  V=(Rz*Ry*Rx*V')';
  m.V=V; m.F=F; m.part=part;
  m.bounds=struct('x_min',min(V(:,1)),'x_max',max(V(:,1)),'y_min',min(V(:,2)),'y_max',max(V(:,2)),'z_min',min(V(:,3)),'z_max',max(V(:,3)));
  V0=m.V(m.F(:,1),:); V1=m.V(m.F(:,2),:); V2=m.V(m.F(:,3),:);
  m.triMin=min(min(V0,V1),V2); m.triMax=max(max(V0,V1),V2);
end

% ==================== mapFragments.m ====================
function mf = mapFragments(frags, p, mesh)
  % mapFragments  Orient real warhead-frame fragments into the engine frame and place the burst point.
  % Input frags from loadFragments (.V Nx3 m/s warhead frame, .mass_kg Nx1, .n). Output engine-frame
  % ejection velocities + a single burst point (per the LOCKED skin-based geometry) + per-fragment mass.
  % Rwe pinned (spec 2026-08-06 fragment-mapping-aiming §3): warhead Z->x, X->y, Y->z (proper rotation, det +1).
  if nargin < 3 || isempty(mesh), mesh = makeUAVmesh(p.uav); end
  Rwe = [0 0 1; 1 0 0; 0 1 0];
  mf.Vej = (Rwe * frags.V.').';            % rotate each row-vector velocity into the engine frame
  mf.mass_kg = frags.mass_kg;
  mf.n = frags.n;
  phi = p.burst.clock_deg * pi/180;        % clock around the drone in the engine y-z plane; 0 = 12 o'clock = +z
  dir = [0, sin(phi), cos(phi)];           % unit direction from drone centre toward the burst
  support = max(mesh.V * dir.');           % farthest skin extent along dir
  mf.burst = dir * (support + p.burst.miss_m);   % 5 m beyond the skin along dir (skin-based miss)
end

% ==================== meshHit.m ====================
function [hit,t,P,part,face_idx] = meshHit(p0,p1,m)
  % meshHit  Earliest segment-vs-mesh intersection across all triangles.
  % Also returns face_idx (the struck triangle index) for struck-area coverage.
  hit=false; t=Inf; P=[NaN NaN NaN]; part=''; face_idx=0;
  for i=1:size(m.F,1)
    A=m.V(m.F(i,1),:); B=m.V(m.F(i,2),:); C=m.V(m.F(i,3),:);
    [h,tt,PP]=segTriHit(p0,p1,A,B,C);
    if h && tt<t, hit=true; t=tt; P=PP; part=m.part{i}; face_idx=i; end
  end
end

% ==================== meshHitCull.m ====================
function [hit,t,P,part,fi] = meshHitCull(p0,p1,m)
  % meshHitCull  Inclusive-AABB broadphase then meshHitVec on survivors. Result-identical to meshHitVec,
  % faster on large meshes. Falls back to computing per-face AABBs if the mesh lacks triMin/triMax.
  if ~isfield(m,'triMin') || ~isfield(m,'triMax')
    V0=m.V(m.F(:,1),:); V1=m.V(m.F(:,2),:); V2=m.V(m.F(:,3),:);
    m.triMin=min(min(V0,V1),V2); m.triMax=max(max(V0,V1),V2);
  end
  smin=min(p0,p1); smax=max(p0,p1);
  cand = find( m.triMax(:,1)>=smin(1) & m.triMin(:,1)<=smax(1) & ...
               m.triMax(:,2)>=smin(2) & m.triMin(:,2)<=smax(2) & ...
               m.triMax(:,3)>=smin(3) & m.triMin(:,3)<=smax(3) );   % inclusive bounds (spec §5)
  if isempty(cand), hit=false; t=Inf; P=[NaN NaN NaN]; part=''; fi=0; return; end
  sub.V=m.V; sub.F=m.F(cand,:); sub.part=m.part(cand);
  [hit,t,P,part,fisub] = meshHitVec(p0,p1,sub);
  if hit, fi=cand(fisub); else, fi=0; end
end

% ==================== meshHitVec.m ====================
function [hit,t,P,part,fi] = meshHitVec(p0,p1,m)
  % meshHitVec  Vectorized batched Moller-Trumbore over ALL faces of m. Same convention as segTriHit
  % (eps0=1e-12, no backface cull, u,v bounds, t in [0,1], P=p0+t*d). Earliest hit; ties -> lowest face idx.
  eps0=1e-12; d=p1-p0;
  V0=m.V(m.F(:,1),:); V1=m.V(m.F(:,2),:); V2=m.V(m.F(:,3),:);
  e1=V1-V0; e2=V2-V0;
  h=[d(2)*e2(:,3)-d(3)*e2(:,2), d(3)*e2(:,1)-d(1)*e2(:,3), d(1)*e2(:,2)-d(2)*e2(:,1)]; % cross(d,e2)
  a=sum(e1.*h,2);
  s=p0-V0;
  q=[s(:,2).*e1(:,3)-s(:,3).*e1(:,2), s(:,3).*e1(:,1)-s(:,1).*e1(:,3), s(:,1).*e1(:,2)-s(:,2).*e1(:,1)]; % cross(s,e1)
  f=1./a;
  U=sum(s.*h,2).*f;
  Vv=(d(1)*q(:,1)+d(2)*q(:,2)+d(3)*q(:,3)).*f;
  T=sum(e2.*q,2).*f;
  ok = (abs(a)>=eps0) & (U>=0) & (U<=1) & (Vv>=0) & (U+Vv<=1) & (T>=0) & (T<=1);
  T(~ok)=Inf;
  [t,fi]=min(T);                     % min returns the FIRST (lowest) index on exact ties
  if isfinite(t)
    hit=true; P=p0+t*d; part=m.part{fi};
  else
    hit=false; t=Inf; P=[NaN NaN NaN]; part=''; fi=0;
  end
end

% ==================== newHeadlessFigure.m ====================
function f = newHeadlessFigure()
  % newHeadlessFigure  An off-screen figure that can be PRINTED headlessly in Octave (--no-gui) AND
  % MATLAB. Octave's default toolkit (fltk) cannot print an invisible figure, so switch THIS figure to
  % gnuplot in Octave only (break-plan #2 - verified fltk errors, gnuplot-on-handle works); MATLAB has
  % no graphics_toolkit and needs no switch.
  f = figure('visible','off');
  if exist('OCTAVE_VERSION','builtin')
    graphics_toolkit(f,'gnuplot');
  end
end

% ==================== plotResults.m ====================
function fig = plotResults(sw, fname)
  % plotResults  Dual-safe 2-D sweeps: A representative miss vs speed; B coverage & density vs fan.
  % Writes a PNG to fname. NOTE: the returned handle is CLOSED after printing (side effect is the file).
  fig=newHeadlessFigure();
  subplot(1,2,1); plot(sw.A.speed, sw.A.rep_miss_x,'-o');
  title('Sweep A: miss vs speed'); xlabel('fragment speed (m/s)'); ylabel('representative miss x (m)');
  subplot(1,2,2); plot(sw.B.fan_deg, sw.B.coverage,'-o', sw.B.fan_deg, sw.B.density/max([sw.B.density 1]),'-x');
  title('Sweep B: coverage & density vs fan'); xlabel('fan half-angle (deg)'); ylabel('coverage / norm. density');
  legend('coverage','density (norm)');
  print(fig,'-dpng',fname);
  close(fig);   % don't leak an invisible figure handle (break-build pass-2 #1)
end

% ==================== plotSchematic.m ====================
function fig = plotSchematic(p, fname)
  % plotSchematic  Dual-safe 2-D top-down schematic: interceptor axis, UAV, the 5 m gap, spray fan.
  % Writes a PNG to fname. NOTE: the returned handle is CLOSED after printing (side effect is the file).
  d=p.datum.radial_offset; hl=p.uav.fuselage_len/2; hs=p.uav.wingspan/2;
  fig=newHeadlessFigure(); hold on;
  plot([-3 3],[0 0],'k-');                                  % target axis (y=0)
  plot([-3 3],[-d -d],'b--');                               % interceptor axis (y=-d)
  plot([-hl hl hl -hl -hl],[-0.17 -0.17 0.17 0.17 -0.17],'k-');   % fuselage box (x by y)
  plot([-0.17 0.17 0.17 -0.17 -0.17],[-hs -hs hs hs -hs],'k-');   % wing (chord by span)
  plot(0,-d,'r.','markersize',20);                          % burst point
  for a=-15:5:15, plot([0 3*sind(a)],[-d -d+3*cosd(a)],'r-'); end % fan lines (+y)
  text(0,-d-0.3,'burst'); text(0,hs+0.3,'UAV');
  title('Interceptor engagement (top-down)'); xlabel('along-track x (m)'); ylabel('across-track y (m)');
  axis equal; hold off;
  print(fig,'-dpng',fname);
  close(fig);   % don't leak an invisible figure handle (break-build pass-2 #1)
end

% ==================== propagateFragments.m ====================
function r = propagateFragments(Vfrag, p, mesh, mass_kg, burst_xyz)
  % propagateFragments  One fragment (Vfrag=[Vx Vy Vz] engine frame). Backward-compatible: calling
  % (Vfrag,p) or (Vfrag,p,mesh) behaves EXACTLY as before. Optional appended args:
  %   mass_kg   (default p.frag.mass_kg)  -- per-fragment mass, so real-data KE is per-fragment.
  %   burst_xyz (default [0,-offset,0])   -- arbitrary burst point; a non-default burst uses a general
  %                                          straight-line traversal (drag-off only; the real-data path).
  % Drag/gravity ON is supported only on the DEFAULT burst (the validated synthetic geometry).
  if nargin < 3 || isempty(mesh), mesh = makeUAVmesh(p.uav); end
  if nargin < 4 || isempty(mass_kg), mass_kg = p.frag.mass_kg; end
  useDefault = (nargin < 5 || isempty(burst_xyz));
  if useDefault, burst_xyz = [0, -p.datum.radial_offset, 0]; end
  % Generalised burst-inside-mesh guard (break-spec #8): warn if the burst sits inside the mesh AABB on
  % ANY axis (the old check only tested the y-extent, valid only for the fixed -y burst).
  b = mesh.bounds;
  if burst_xyz(1)>=b.x_min && burst_xyz(1)<=b.x_max && burst_xyz(2)>=b.y_min && burst_xyz(2)<=b.y_max ...
     && burst_xyz(3)>=b.z_min && burst_xyz(3)<=b.z_max
    warning('propagateFragments:burstInsideMesh', ...
      'burst point [%.3g %.3g %.3g] lies inside the mesh AABB; burst-outside-mesh assumption violated.', ...
      burst_xyz(1), burst_xyz(2), burst_xyz(3));
  end
  if p.phys.drag || p.phys.gravity
    if ~useDefault
      error('propagateFragments: general-burst (real-data) path with drag/gravity is not supported yet; run drag off');
    end
    r = integrateFragment(Vfrag, p, mesh, mass_kg); return;
  end
  % ---- straight-line baseline (drag & gravity off) ----
  Vi=p.interceptor.speed; Vt=p.target.speed;
  vrel=[Vi+Vfrag(1)+Vt, Vfrag(2), Vfrag(3)];
  r=struct('hit',false,'noncrossing',false,'timeout',false,'part','','face_idx',0,'t_s',NaN,'P',[NaN NaN NaN],'KE_J',NaN,'miss_x',NaN,'miss_z',NaN);
  if useDefault
    d=p.datum.radial_offset;
    if vrel(2)<=0, r.noncrossing=true; return; end
    tc=d/vrel(2); r.miss_x=vrel(1)*tc; r.miss_z=vrel(3)*tc;
    p0=[0 -d 0];
    Tmax=(mesh.bounds.y_max + 0.1 - p0(2))/vrel(2);   % time to pass y_max+eps
  else
    p0=burst_xyz;
    sp=norm(vrel); if sp<=0, r.noncrossing=true; return; end
    reach=max(sqrt(sum((mesh.V - repmat(p0,size(mesh.V,1),1)).^2,2))) + 0.1;  % farthest vertex + margin
    Tmax=reach/sp;
  end
  p1=p0 + vrel*Tmax;
  [hit,tfrac,P,part,face_idx]=meshHitCull(p0,p1,mesh);
  if hit
    r.hit=true; r.part=part; r.face_idx=face_idx; r.t_s=tfrac*Tmax; r.P=P;
    r.KE_J=0.5*mass_kg*sum(vrel.^2);                 % |v_rel| constant (no drag) -> impact speed
  end
end

% ==================== segTriHit.m ====================
function [hit,t,P] = segTriHit(p0,p1,A,B,C)
  % segTriHit  Moller-Trumbore: does segment p0->p1 cross triangle ABC? t in [0,1].
  hit=false; t=Inf; P=[NaN NaN NaN]; eps0=1e-12;
  d=p1-p0; e1=B-A; e2=C-A; h=cross(d,e2); a=dot(e1,h);
  if abs(a)<eps0, return; end
  f=1/a; s=p0-A; u=f*dot(s,h);
  if u<0 || u>1, return; end
  q=cross(s,e1); v=f*dot(d,q);
  if v<0 || u+v>1, return; end
  tt=f*dot(e2,q);
  if tt>=0 && tt<=1, hit=true; t=tt; P=p0+tt*d; end
end

% ==================== stepState.m ====================
function [pos2, vrel2] = stepState(pos, vrel, dt, p)
  % stepState  One integration step of (pos, v_rel) under fragAccel. Euler or RK4.
  % RK4 re-evaluates fragAccel (hence v_air) at EACH stage from that stage's v_rel (break-spec-2 #3).
  switch p.phys.integrator
    case 'euler'
      a = fragAccel(vrel, p);
      pos2  = pos  + vrel*dt;
      vrel2 = vrel + a*dt;
    case 'rk4'
      k1v = fragAccel(vrel, p);               k1p = vrel;
      k2v = fragAccel(vrel + 0.5*dt*k1v, p);  k2p = vrel + 0.5*dt*k1v;
      k3v = fragAccel(vrel + 0.5*dt*k2v, p);  k3p = vrel + 0.5*dt*k2v;
      k4v = fragAccel(vrel + dt*k3v, p);       k4p = vrel + dt*k3v;
      pos2  = pos  + (dt/6)*(k1p + 2*k2p + 2*k3p + k4p);
      vrel2 = vrel + (dt/6)*(k1v + 2*k2v + 2*k3v + k4v);
    otherwise
      error('stepState: unknown integrator "%s"', p.phys.integrator);
  end
end

% ==================== summaryReport.m ====================
function txt = summaryReport(p, res, cov, db, tm)
  % summaryReport  Plain-language engagement summary (dual-safe char). Honest per data source: the
  % synthetic head-on path prints its timing/delay-budget/fire-early analysis; the REAL-DATA path prints
  % only what the real burst geometry actually supports, and flags the placeholder mesh + the synthetic-
  % only figures it deliberately omits (break-build pass-2 #1,#2).
  L = {};
  realpath = strcmp(p.frag.source,'file');
  if realpath, srclabel = sprintf('REAL DATA (hydrocode fragments, N=%d)', cov.n_total);
  else,        srclabel = upper(p.frag.source); end
  L{end+1}=sprintf('INTERCEPTOR ENGAGEMENT SUMMARY  (fragment data: %s)', srclabel);
  if realpath
    L{end+1}=sprintf('  Interceptor %.1f m/s | UAV %.1f m/s | closing %.1f m/s | burst %d o''clock, %.1f m from skin', ...
      p.interceptor.speed, p.target.speed, db.closing_speed, clock12(p.burst.clock_deg), p.burst.miss_m);
  else
    L{end+1}=sprintf('  Interceptor %.1f m/s | UAV %.1f m/s | closing %.1f m/s | offset %.1f m', ...
      p.interceptor.speed, p.target.speed, db.closing_speed, p.datum.radial_offset);
    L{end+1}=sprintf('  Electronics window %.0f-%.0f us | representative fragment flight t3 = %.2f ms', ...
      db.elec_delay_min_s*1e6, db.elec_delay_max_s*1e6, db.t3_s*1e3);
    L{end+1}=sprintf('  Delay budget: sensing1 %.1f%% | sensing2 %.1f%% | fragment %.1f%%', ...
      db.budget_pct(1), db.budget_pct(2), db.budget_pct(3));
  end
  L{end+1}=sprintf('  Impacts %d of %d (%.0f%%) | fuselage %d wing %d tail %d | non-crossing %d', ...
    cov.n_hits, cov.n_total, 100*cov.hit_fraction, cov.n_fuselage, cov.n_wing, cov.n_tail, cov.n_noncrossing);
  L{end+1}=sprintf('  Kinetic energy: total %.1f J | per hit %.1f J  (%s)', ...
    cov.KE_total_J, cov.KE_per_hit_J, ternary(p.phys.drag,'with drag - defensible','no drag - UPPER BOUND'));
  L{end+1}=sprintf('  Coverage %.3f of mesh | density %.1f hits/m2', cov.coverage, cov.density_hits_per_m2);
  if realpath
    if strcmp(p.uav.mesh_source,'stl')
      L{end+1}=sprintf('  NOTE: the REAL Predator STL mesh is wired into the engine. A low hit count with a small');
      L{end+1}=sprintf('        fragment sample is a SAMPLING artifact (a thin radial belt vs a thin target, ~half sprays');
      L{end+1}=sprintf('        away) - sweep clock_deg and use the full fragment file for real coverage. Timing /');
      L{end+1}=sprintf('        delay-budget / fire-early figures are OMITTED: they belong to the synthetic head-on datum.');
      L{end+1}=sprintf('        Part labels (fuselage/wing/tail) are a geometric-centroid heuristic, not a CAD breakdown.');
    else
      L{end+1}=sprintf('  NOTE: the drone mesh is a CRUDE PLACEHOLDER (real Predator STL not wired in for this run -');
      L{end+1}=sprintf('        set uav.mesh_source=stl), so the hit count is a placeholder-geometry artifact and is very');
      L{end+1}=sprintf('        sensitive to the burst clock position - sweep clock_deg. Timing / delay-budget / fire-early');
      L{end+1}=sprintf('        figures are OMITTED here: they belong to the synthetic head-on datum, not this real burst.');
    end
  else
    L{end+1}=sprintf('  Electronics delay: target displacement %.3f-%.3f m  vs  along-track closure %.3f-%.3f m  (~%.1fx)', ...
      tm.target_drift_x(1), tm.target_drift_x(2), tm.drift_x(1), tm.drift_x(2), tm.drift_x(1)/tm.target_drift_x(1));
    if tm.has_pattern
      L{end+1}=sprintf('  Forward offset: fragment-carry %.3f m + electronics-closure %.3f-%.3f m', ...
        tm.carry_x, tm.drift_x(1), tm.drift_x(2));
      L{end+1}=sprintf('  Fire-early lead %.3f-%.3f m  (%.3f-%.3f ms) | ideal: centroid on drone', ...
        tm.lead_distance(1), tm.lead_distance(2), tm.lead_time(1)*1e3, tm.lead_time(2)*1e3);
    else
      L{end+1}=sprintf('  Forward offset / fire-early lead: N/A (no crossing fragments)');
    end
    if p.phys.drag
      L{end+1}=sprintf('  (offset & lead are drag-independent geometry - the rigid translation holds under drag; only KE reflects drag)');
    end
  end
  txt = strjoin(L, sprintf('\n'));
end

function c = clock12(deg)
  c = mod(round(deg/30),12); if c==0, c=12; end
end

function s = ternary(c, a, b)
  if c, s=a; else, s=b; end
end

% ==================== sweeps.m ====================
function sw = sweeps(p)
  % sweeps  Two engagement sweeps (data only; spec 10e). Forces drag/gravity OFF regardless of p (perf:
  % it regenerates+propagates the set 16x; trends are clearest drag-off and drag only scales KE ~uniformly).
  %   A: fragment speed 800->1200 (monodisperse) -> representative miss, hit-count, mean KE.
  %   B: eject fan half-angle 0->30 deg -> coverage, density, along-track spread (coverage-vs-density).
  mesh = makeUAVmesh(p.uav);
  speeds = 800:50:1200;
  A.speed=speeds; A.rep_miss_x=zeros(size(speeds)); A.n_hits=zeros(size(speeds)); A.mean_KE=zeros(size(speeds));
  for i=1:numel(speeds)
    q=p; q.frag.speed_range=[speeds(i) speeds(i)]; q.phys.drag=false; q.phys.gravity=false;
    V=syntheticFragments(q);
    res=arrayfun(@(k) propagateFragments(V(k,:),q,mesh),(1:size(V,1))');
    hit=[res.hit]; A.n_hits(i)=sum(hit);
    if any(hit), A.mean_KE(i)=mean([res(hit).KE_J]); end
    rr=propagateFragments([0 speeds(i) 0], q, mesh);   % representative perpendicular fragment
    A.rep_miss_x(i)=rr.miss_x;
  end
  fans = 0:5:30;
  B.fan_deg=fans; B.coverage=zeros(size(fans)); B.density=zeros(size(fans)); B.spread_x=zeros(size(fans));
  for i=1:numel(fans)
    q=p; q.frag.eject_range_deg=[-fans(i) fans(i)]; q.phys.drag=false; q.phys.gravity=false;
    V=syntheticFragments(q);
    res=arrayfun(@(k) propagateFragments(V(k,:),q,mesh),(1:size(V,1))');
    cov=coverageLethality(res, mesh);
    B.coverage(i)=cov.coverage; B.density(i)=cov.density_hits_per_m2;
    hit=[res.hit]; if sum(hit)>1, Ps=vertcat(res(hit).P); B.spread_x(i)=std(Ps(:,1)); end
  end
  sw.A=A; sw.B=B;
end

% ==================== syntheticFragments.m ====================
function V = syntheticFragments(p)
  % syntheticFragments  Seeded Nx3 (Vx,Vy,Vz) in the INTERCEPTOR frame (Vx centred ~0).
  % Per fragment: speed in speed_range, eject psi (x-y from +y), vertical zeta.
  %   Vx=Vsin(psi)cos(zeta), Vy=Vcos(psi)cos(zeta), Vz=Vsin(zeta)  -> |V|=speed, Vy>0.
  % Uses lcgUniform (self-contained) so the set is identical in Octave AND MATLAB and
  % does NOT touch the global rand stream (break-build pass-2 findings #1,#2).
  n = p.frag.count;
  U = lcgUniform(3*n, p.frag.seed);      % one continuous deterministic stream
  u1 = U(1:n); u2 = U(n+1:2*n); u3 = U(2*n+1:3*n);
  spd  = p.frag.speed_range(1) + diff(p.frag.speed_range)*u1;
  psi  = (p.frag.eject_range_deg(1) + diff(p.frag.eject_range_deg)*u2) * pi/180;
  zeta = (-p.frag.z_spread_deg + 2*p.frag.z_spread_deg*u3) * pi/180;
  V = [ spd.*sin(psi).*cos(zeta), spd.*cos(psi).*cos(zeta), spd.*sin(zeta) ];
end

% ==================== timingModes.m ====================
function tm = timingModes(results, p)
  % timingModes  Along-track pattern offset (realistic vs ideal) + fire-early lead. Data only, no
  % figures. Target frame; +x = downstream (aft of the drone). Two offset components reported
  % separately (spec 10a): fragment-flight carry (pattern centroid, burst at datum) and electronics
  % drift = closing*tau. Lead nulls the total centroid offset; denominator is the CLOSING speed (spec 9).
  Vi=p.interceptor.speed; Vt=p.target.speed; closing=Vi+Vt;
  % Pattern centroid = mean diagnostic along-track impact over ALL CROSSING fragments (Vy>0), NOT only
  % those that strike the small mesh (break-plan #1: hit-only is survivorship-biased ~40% low and
  % contradicts spec's ~1.1 m anchor). miss_x is the diagnostic plane point set in propagateFragments.
  crossing = ~[results.noncrossing];
  mx = [results(crossing).miss_x];
  mx = mx(~isnan(mx));                      % real-data general-burst path leaves miss_x NaN -> no +y pattern
  tm.has_pattern = ~isempty(mx);           % false if no crossing pattern (guards NaN in summary; real path -> N/A)
  if tm.has_pattern, carry_x = mean(mx); carry_sd = std(mx); else, carry_x = NaN; carry_sd = NaN; end
  tau = [p.delays.sensing1(1)+p.delays.sensing2(1), p.delays.sensing1(2)+p.delays.sensing2(2)]; % [min max]
  tm.closing_speed = closing;
  tm.carry_x = carry_x;                    % fragment-flight carry centroid (~1.22 m for defaults)
  tm.carry_sd = carry_sd;                  % along-track spread (unchanged by a rigid lead translation)
  tm.drift_x = closing * tau;              % electronics-delay ALONG-TRACK CLOSURE = closing*tau [min max]
  tm.target_drift_x = p.target.speed * tau;% electronics-delay TARGET displacement = target*tau (~2.6x smaller; spec 10a)
  tm.realistic_offset = carry_x + tm.drift_x;   % total centroid offset [min max]
  tm.ideal_offset = 0;                     % best case: pattern centroid on the drone
  tm.lead_distance = tm.realistic_offset;  % fire-early distance [min max] = the offset to null
  tm.lead_time = tm.realistic_offset ./ closing;  % [min max] s; denominator = closing speed
end

