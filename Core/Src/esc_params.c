#include "esc_params.h"
#include "esc_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PARAMS_DEFAULTS_INIT {                                                           \
    .pole_pairs = DEF_POLE_PAIRS, .six_step = DEF_SIX_STEP, .dir = 1, .auto_mode = DEF_AUTO, \
    .kv = DEF_KV, .rpm = DEF_RPM, .accel = DEF_ACCEL, .boost_v = DEF_BOOST_V,            \
    .max_mod = DEF_MAX_MOD, .ilim = DEF_ILIM_A, .align_v = DEF_ALIGN_V,                  \
    .align_ms = DEF_ALIGN_MS, .r_ll_mohm = DEF_RLL_MOHM, .l_ll_uh = DEF_LLL_UH,          \
    .kt_nm_a = DEF_KT, .i_drive = DEF_IDRIVE, .rpm_max = DEF_RPM_MAX,   \
    .cur_bw = DEF_CUR_BW, .cur_kp = DEF_CUR_KP, .cur_ki = DEF_CUR_KI,                    \
    .spd_kp = DEF_SPD_KP, .spd_ki = DEF_SPD_KI,                                          \
    .pos_kp = DEF_POS_KP, .pos_ki = DEF_POS_KI, .pos_kd = DEF_POS_KD,                    \
    .pos_vmax = DEF_POS_VMAX, .pos_acc = DEF_POS_ACC, .i_max = DEF_IMAX,                 \
    .i_align = DEF_IALIGN, .i_cal = DEF_ICAL, .ot_drv_c = DEF_OT_DRV_C, .ot_mot_c = DEF_OT_MOT_C, \
    .cur_auto = 1, .ot_drv_en = DEF_OT_DRV_EN, .ot_mot_en = DEF_OT_MOT_EN, .enc_dir = 0, \
    .preset_name = DEF_PRESET_NAME }

esc_params_t g_par = PARAMS_DEFAULTS_INIT;
esc_eff_t    g_eff = { 0.0f, 0.0f, 0.03f, 0.0f, 0.05f, 100.0f };

_Static_assert(sizeof(esc_params_t) == 144, "esc_params_t layout changed: bump PARAMS_VERSION and the GUI");
_Static_assert(sizeof(esc_saved_t) == 176, "esc_saved_t layout changed");

void params_defaults(esc_params_t *p)
{
    static const esc_params_t d = PARAMS_DEFAULTS_INIT;
    *p = d;
}

/* ------------------------------------------------------------------ descriptor table */
typedef enum { PT_U8, PT_I8, PT_BOOL, PT_U32, PT_F32, PT_I8Z } ptype_t;   /* PT_I8Z: -1, 0 or +1 */
typedef struct { const char *name; ptype_t type; uint16_t off; float min, max; } pdesc_t;

#define D(n, t, f, lo, hi) { n, t, (uint16_t)offsetof(esc_params_t, f), (float)(lo), (float)(hi) }
static const pdesc_t tbl[] = {
    D("pp",      PT_U8,   pole_pairs, 1,     50),
    D("six",     PT_BOOL, six_step,   0,     1),
    D("dir",     PT_I8,   dir,        -1,    1),
    D("auto",    PT_BOOL, auto_mode,  0,     1),
    D("kv",      PT_F32,  kv,         1,     5000),
    D("rpm",     PT_F32,  rpm,        0,     10000),
    D("accel",   PT_F32,  accel,      1,     10000),
    D("boost",   PT_F32,  boost_v,    0,     30),
    D("maxmod",  PT_F32,  max_mod,    0.01f, 0.95f),
    D("ilim",    PT_F32,  ilim,       0.5f,  100),
    D("alignv",  PT_F32,  align_v,    0,     30),
    D("alignms", PT_U32,  align_ms,   0,     10000),
    D("rl",      PT_F32,  r_ll_mohm,  1,     10000),
    D("ll",      PT_F32,  l_ll_uh,    0,     10000),
    D("kt",      PT_F32,  kt_nm_a,    0.001f, 10),
    D("idrive",  PT_F32,  i_drive,    0.1f,  100),
    D("rpmmax",  PT_F32,  rpm_max,    1,     20000),
    D("curbw",   PT_F32,  cur_bw,     10,    5000),
    D("curkp",   PT_F32,  cur_kp,     0,     100),
    D("curki",   PT_F32,  cur_ki,     0,     100000),
    D("curauto", PT_BOOL, cur_auto,   0,     1),
    D("spdkp",   PT_F32,  spd_kp,     0,     10),
    D("spdki",   PT_F32,  spd_ki,     0,     1000),
    D("poskp",   PT_F32,  pos_kp,     0,     1000),
    D("poski",   PT_F32,  pos_ki,     0,     1000),
    D("poskd",   PT_F32,  pos_kd,     0,     100),
    D("posvmax", PT_F32,  pos_vmax,   1,     20000),
    D("posacc",  PT_F32,  pos_acc,    1,     100000),
    D("imax",    PT_F32,  i_max,      0.1f,  100),
    D("ialign",  PT_F32,  i_align,    0,     100),
    D("ical",    PT_F32,  i_cal,      0.2f,  100),
    D("otdrv",   PT_F32,  ot_drv_c,   20,    150),
    D("otdrven", PT_BOOL, ot_drv_en,  0,     1),
    D("otmot",   PT_F32,  ot_mot_c,   20,    200),
    D("otmoten", PT_BOOL, ot_mot_en,  0,     1),
    D("encdir",  PT_I8Z,  enc_dir,    -1,    1),
};
#define NPARAMS ((int)(sizeof(tbl) / sizeof(tbl[0])))

int params_count(void) { return NPARAMS; }
const char *params_name(int idx) { return (idx >= 0 && idx < NPARAMS) ? tbl[idx].name : NULL; }

static const pdesc_t *find(const char *name)
{
    for (int i = 0; i < NPARAMS; i++)
        if (strcmp(tbl[i].name, name) == 0) return &tbl[i];
    return NULL;
}

static float read_val(const esc_params_t *p, const pdesc_t *d)
{
    const uint8_t *b = (const uint8_t *)p + d->off;
    switch (d->type) {
    case PT_U8:
    case PT_BOOL: return (float)*b;
    case PT_I8:
    case PT_I8Z:  return (float)*(const int8_t *)b;
    case PT_U32: { uint32_t u; memcpy(&u, b, 4); return (float)u; }
    default:     { float f; memcpy(&f, b, 4); return f; }
    }
}

static bool in_range(const pdesc_t *d, float v)
{
    if (d->type == PT_BOOL) return true;
    if (d->type == PT_I8 && v == 0.0f) return false;
    return (v >= d->min && v <= d->max);          /* false for NaN */
}

bool params_get(const esc_params_t *p, const char *name, float *v)
{
    const pdesc_t *d = find(name);
    if (!d) return false;
    *v = read_val(p, d);
    return true;
}

bool params_set_name(esc_params_t *p, const char *name)
{
    size_t n = strlen(name);
    if (n < 1 || n >= PRESET_NAME_LEN) return false;
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)name[i] < 0x20 || (unsigned char)name[i] > 0x7E || name[i] == ',') return false;
    memset(p->preset_name, 0, sizeof(p->preset_name));
    memcpy(p->preset_name, name, n);
    return true;
}

int params_format_name(const esc_params_t *p, char *buf, size_t n)
{
    return snprintf(buf, n, "N,%.*s\r\n", PRESET_NAME_LEN - 1, p->preset_name);
}

bool params_set(esc_params_t *p, const char *name, float v)
{
    const pdesc_t *d = find(name);
    if (!d || !in_range(d, v)) return false;
    uint8_t *b = (uint8_t *)p + d->off;
    switch (d->type) {
    case PT_U8:   *b = (uint8_t)(v + 0.5f); break;
    case PT_BOOL: *b = (v != 0.0f) ? 1u : 0u; break;
    case PT_I8:   *(int8_t *)b = (v < 0.0f) ? -1 : 1; break;
    case PT_I8Z:  *(int8_t *)b = (v < -0.5f) ? -1 : ((v > 0.5f) ? 1 : 0); break;
    case PT_U32:  { uint32_t u = (uint32_t)(v + 0.5f); memcpy(b, &u, 4); break; }
    default:      memcpy(b, &v, 4); break;
    }
    return true;
}

int params_format(const esc_params_t *p, int idx, char *buf, size_t n)
{
    if (idx < 0 || idx >= NPARAMS) return 0;
    const pdesc_t *d = &tbl[idx];
    const uint8_t *b = (const uint8_t *)p + d->off;
    switch (d->type) {
    case PT_U8:
    case PT_BOOL: return snprintf(buf, n, "P,%s,%u\r\n", d->name, (unsigned)*b);
    case PT_I8:
    case PT_I8Z:  return snprintf(buf, n, "P,%s,%d\r\n", d->name, (int)*(const int8_t *)b);
    case PT_U32:  { uint32_t u; memcpy(&u, b, 4); return snprintf(buf, n, "P,%s,%lu\r\n", d->name, (unsigned long)u); }
    default: {
        float f; memcpy(&f, b, 4);
        /* plain long: the nano printf of the toolchain has no %lld. Largest value (100000) x 10000 still fits in 32 bit. */
        long m = (long)(f * 10000.0f + (f < 0 ? -0.5f : 0.5f));
        const char *sg = (m < 0) ? "-" : "";
        if (m < 0) m = -m;
        return snprintf(buf, n, "P,%s,%s%ld.%04ld\r\n", d->name, sg, m / 10000, m % 10000);
    }
    }
}

static bool params_valid(const esc_params_t *p)
{
    bool term = false;                                        /* name must be terminated and printable */
    for (int i = 0; i < PRESET_NAME_LEN; i++) {
        unsigned char c = (unsigned char)p->preset_name[i];
        if (c == 0) { term = true; break; }
        if (c < 0x20 || c > 0x7E) return false;
    }
    if (!term) return false;
    for (int i = 0; i < NPARAMS; i++) {
        float v = read_val(p, &tbl[i]);
        if (tbl[i].type == PT_BOOL) { if (v != 0.0f && v != 1.0f) return false; continue; }
        if (!in_range(&tbl[i], v)) return false;
    }
    return true;
}

/* ------------------------------------------------------------------ derived values */
void params_derive(const esc_params_t *p, float vbus, esc_eff_t *e)
{
    {   /* current loop gains: bandwidth alpha gives kp = alpha L, ki = alpha R (phase values = half the line-line ones) */
        float ls = p->l_ll_uh * 0.5e-6f;
        if (ls < 1.0e-6f) ls = 1.0e-6f;
        float rs = p->r_ll_mohm * 0.5e-3f;
        float al = 6.2831853f * p->cur_bw;
        e->cur_kp = p->cur_auto ? al * ls : p->cur_kp;
        e->cur_ki = p->cur_auto ? al * rs : p->cur_ki;
    }
    float vb = (vbus < 5.0f) ? 5.0f : vbus;
    if (p->auto_mode) {
        /* boost = voltage for the wanted current across the winding + one leg's worth of dead-time loss */
        float r_ll = p->r_ll_mohm * 0.001f;
        float v_dt = vb * ((float)DEADTIME_NS * 1e-9f) * (float)PWM_FREQ_HZ;
        float boost = p->i_drive * r_ll + v_dt;
        float rpm_t = fminf(fabsf(p->rpm), p->rpm_max);
        float v_need = rpm_t / p->kv + boost;
        float m = 1.1547f * v_need * 1.3f / vb;               /* 30 % margin above what is needed */
        if (m < 0.03f) m = 0.03f;
        if (m > 0.95f) m = 0.95f;
        e->boost_v = boost;
        e->align_v = boost;
        e->max_mod = m;
        e->rpm_limit = fminf(p->rpm_max, params_max_rpm(p, vb));
    } else {
        e->boost_v = p->boost_v;
        e->align_v = p->align_v;
        e->max_mod = p->max_mod;
        e->rpm_limit = p->rpm_max;
    }
}

float params_max_rpm(const esc_params_t *p, float vbus)
{
    float vb = (vbus < 5.0f) ? 5.0f : vbus;
    float r_ll = p->r_ll_mohm * 0.001f;
    float boost = p->i_drive * r_ll + vb * ((float)DEADTIME_NS * 1e-9f) * (float)PWM_FREQ_HZ;
    float v_avail = 0.9f * vb / 1.1547f;                       /* peak line-line at 90 % modulation */
    float r = (v_avail - boost) * p->kv;
    return (r > 0.0f) ? r : 0.0f;
}

/* ------------------------------------------------------------------ flash image */
uint32_t params_crc32(const void *data, size_t n)
{
    const uint8_t *b = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= b[i];
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

void params_pack(esc_saved_t *s, uint32_t flags, const esc_params_t *p, const float off[3])
{
    memset(s, 0, sizeof(*s));
    s->magic = PARAMS_MAGIC;
    s->version = PARAMS_VERSION;
    s->flags = flags & (SAVED_FLAG_PAR | SAVED_FLAG_CAL);
    if (s->flags & SAVED_FLAG_PAR) s->par = *p;
    if (s->flags & SAVED_FLAG_CAL) memcpy(s->cs_offset, off, sizeof(s->cs_offset));
    s->crc = params_crc32(s, offsetof(esc_saved_t, crc));
}

static bool offsets_valid(const float off[3])
{
    for (int i = 0; i < 3; i++)
        if (!(off[i] >= ZERO_MIN_COUNTS && off[i] <= ZERO_MAX_COUNTS)) return false;      /* also rejects NaN */
    return true;
}

bool params_unpack(const esc_saved_t *s, uint32_t *flags, esc_params_t *p, float off[3])
{
    if (s->magic != PARAMS_MAGIC || s->version != PARAMS_VERSION) return false;
    if (s->crc != params_crc32(s, offsetof(esc_saved_t, crc))) return false;
    if ((s->flags & SAVED_FLAG_PAR) && !params_valid(&s->par)) return false;
    if ((s->flags & SAVED_FLAG_CAL) && !offsets_valid(s->cs_offset)) return false;
    *flags = s->flags;
    if (s->flags & SAVED_FLAG_PAR) *p = s->par;
    if (s->flags & SAVED_FLAG_CAL) memcpy(off, s->cs_offset, sizeof(s->cs_offset));
    return true;
}
