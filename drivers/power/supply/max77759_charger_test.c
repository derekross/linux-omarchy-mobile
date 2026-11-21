// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the battery temperature guard of the MAX77759 charger
 * driver: bands, hysteresis, tiers, limits and the tables themselves; the
 * charge thresholds and the read-back of the limits.
 *
 * This file is included by max77759_charger.c and can not be built alone.
 */

#include <kunit/device.h>
#include <kunit/test.h>

/* Temperatures are in 0.1 C, as the power supply class reports them */
static void guard_test_band_edges(struct kunit *test)
{
	static const struct {
		int decic;
		int band;
	} c[] = {
		{   -1, -1 }, {   0,  0 }, {   1,  0 },
		{   99,  0 }, { 100,  1 }, { 101,  1 },
		{  199,  1 }, { 200,  2 }, { 201,  2 },
		{  419,  2 }, { 420,  3 }, { 421,  3 },
		{  459,  3 }, { 460,  4 }, { 461,  4 },
		{  479,  4 }, { 480,  5 }, { 481,  5 },
		{  549,  5 }, { 550, -1 }, { 551, -1 },
		{ -200, -1 }, { 250,  2 }, { 900, -1 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(c); i++)
		KUNIT_EXPECT_EQ_MSG(test, guard_classify(c[i].decic), c[i].band,
				    "at %d", c[i].decic);
}

static void guard_test_hysteresis(struct kunit *test)
{
	static const struct {
		int cur;
		int decic;
		int next;
	} c[] = {
		/* The first reading (-2) is taken as it is */
		{ -2, 250,  2 }, { -2, 200,  2 }, { -2, 460,  4 },
		{ -2,  -1, -1 }, { -2, 550, -1 },
		/* No change inside the band */
		{  2, 200,  2 }, {  2, 419,  2 },
		/* A more restrictive band at once */
		{  2, 199,  1 }, {  2, 420,  3 }, {  1,  99,  0 },
		{  3, 460,  4 }, {  4, 480,  5 },
		{  0,  -1, -1 }, {  5, 550, -1 },
		/* A less restrictive one only 2.0 C inside it on both sides */
		{  1, 200,  1 }, {  1, 219,  1 }, {  1, 220,  2 },
		{  0, 100,  0 }, {  0, 119,  0 }, {  0, 120,  1 },
		{  3, 419,  3 }, {  3, 400,  3 }, {  3, 399,  2 },
		/* Out of the inhibit, cold and hot */
		{ -1,   0, -1 }, { -1,  19, -1 }, { -1,  20,  0 },
		{ -1, 549, -1 }, { -1, 530, -1 }, { -1, 529,  5 },
		/*
		 * 42-46 C and 46-48 C are no wider than twice the
		 * hysteresis: cooling never relaxes into them.
		 */
		{  5, 470,  5 }, {  5, 440,  5 }, {  5, 400,  5 },
		{  5, 399,  2 },
		/* Between bands that are not neighbours */
		{  2,  50,  0 }, {  2, 500,  5 }, {  4,  50,  0 },
		{  0, 250,  2 }, {  5, 250,  2 }, {  0, 205,  0 },
	};
	static const struct guard_table *const tbl[] = {
		&guard_table_default, &guard_table_husky,
	};
	int i, t;

	for (t = 0; t < ARRAY_SIZE(tbl); t++)
		for (i = 0; i < ARRAY_SIZE(c); i++)
			KUNIT_EXPECT_EQ_MSG(test,
					    guard_next_band(tbl[t], c[i].cur,
							    c[i].decic),
					    c[i].next,
					    "table %d: band %d at %d", t,
					    c[i].cur, c[i].decic);
}

static void guard_test_top_tier(struct kunit *test)
{
	static const int def[GUARD_NBANDS] = { 1, 2, 2, 2, 1, 0 };
	static const int husky[GUARD_NBANDS] = { 1, 3, 3, 3, 1, 0 };
	int b;

	for (b = 0; b < GUARD_NBANDS; b++) {
		KUNIT_EXPECT_EQ_MSG(test, guard_top_tier(&guard_table_default, b),
				    def[b], "band %d", b);
		KUNIT_EXPECT_EQ_MSG(test, guard_top_tier(&guard_table_husky, b),
				    husky[b], "band %d", b);
	}
}

struct guard_tier_case {
	int band;
	int tier;
	int uv;
	int next;
};

static void guard_test_tiers(struct kunit *test, const struct guard_table *tbl,
			     const struct guard_tier_case *c, int n)
{
	int i;

	for (i = 0; i < n; i++)
		KUNIT_EXPECT_EQ_MSG(test,
				    guard_next_tier(tbl, c[i].band, c[i].tier,
						    c[i].uv),
				    c[i].next, "band %d tier %d at %d uV",
				    c[i].band, c[i].tier, c[i].uv);
}

static void guard_test_tier_steps_default(struct kunit *test)
{
	static const struct guard_tier_case c[] = {
		{ 2, 0, 3700000, 0 }, { 2, 0, 4199999, 0 },
		{ 2, 0, 4200000, 1 }, { 2, 0, 4299999, 1 },
		{ 2, 0, 4300000, 2 }, { 2, 0, 4450000, 2 },
		{ 2, 1, 4299999, 1 }, { 2, 1, 4300000, 2 },
		/* A tier does not fall when the voltage does */
		{ 2, 1, 4100000, 1 }, { 2, 2, 3700000, 2 },
		/* Not above the highest tier the band charges in */
		{ 0, 0, 4199999, 0 }, { 0, 0, 4400000, 1 },
		{ 4, 0, 4400000, 1 }, { 5, 0, 4400000, 0 },
		{ 1, 0, 4400000, 2 }, { 3, 0, 4400000, 2 },
	};

	guard_test_tiers(test, &guard_table_default, c, ARRAY_SIZE(c));
}

static void guard_test_tier_steps_husky(struct kunit *test)
{
	static const struct guard_tier_case c[] = {
		{ 2, 0, 3700000, 0 }, { 2, 0, 4199999, 0 },
		{ 2, 0, 4200000, 1 }, { 2, 0, 4299999, 1 },
		{ 2, 0, 4300000, 2 }, { 2, 0, 4349999, 2 },
		{ 2, 0, 4350000, 3 }, { 2, 0, 4480000, 3 },
		{ 2, 2, 4349999, 2 }, { 2, 2, 4350000, 3 },
		{ 2, 1, 4100000, 1 }, { 2, 3, 3700000, 3 },
		{ 0, 0, 4199999, 0 }, { 0, 0, 4400000, 1 },
		{ 4, 0, 4400000, 1 }, { 5, 0, 4400000, 0 },
		{ 1, 0, 4400000, 3 }, { 3, 0, 4400000, 3 },
	};

	guard_test_tiers(test, &guard_table_husky, c, ARRAY_SIZE(c));
}

static void guard_test_table(struct kunit *test, const struct guard_table *tbl,
			     int ntiers, const u32 *cv_mv,
			     const u8 (*cc_pct)[GUARD_MAX_TIERS])
{
	int b, t;

	KUNIT_ASSERT_EQ(test, tbl->ntiers, ntiers);
	KUNIT_ASSERT_LE(test, tbl->ntiers, GUARD_MAX_TIERS);
	for (t = 0; t < GUARD_MAX_TIERS; t++)
		KUNIT_EXPECT_EQ_MSG(test, tbl->cv_mv[t], cv_mv[t], "tier %d", t);
	for (b = 0; b < GUARD_NBANDS; b++)
		for (t = 0; t < GUARD_MAX_TIERS; t++)
			KUNIT_EXPECT_EQ_MSG(test, tbl->cc_pct[b][t], cc_pct[b][t],
					    "band %d tier %d", b, t);
}

/* The numbers the driver had before it had more than one table */
static void guard_test_default_table(struct kunit *test)
{
	static const u32 cv_mv[GUARD_MAX_TIERS] = { 4200, 4300, 4450 };
	static const u8 cc_pct[GUARD_NBANDS][GUARD_MAX_TIERS] = {
		{  30, 10,  0 },
		{  50, 30, 30 },
		{ 100, 70, 50 },
		{  80, 50, 50 },
		{  50, 50,  0 },
		{  30,  0,  0 },
	};

	guard_test_table(test, &guard_table_default, 3, cv_mv, cc_pct);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[0], 0);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[1], 100);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[2], 200);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[3], 420);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[4], 460);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[5], 480);
	KUNIT_EXPECT_EQ(test, guard_temp_decic[6], 550);
}

static void guard_test_husky_table(struct kunit *test)
{
	static const u32 cv_mv[GUARD_MAX_TIERS] = { 4200, 4300, 4350, 4480 };
	static const u8 cc_pct[GUARD_NBANDS][GUARD_MAX_TIERS] = {
		{  50,  30,   0,  0 },
		{  70,  30,  30, 30 },
		{ 120, 120, 120, 50 },
		{ 100, 100, 100, 50 },
		{  50,  50,   0,  0 },
		{  30,   0,   0,  0 },
	};

	guard_test_table(test, &guard_table_husky, 4, cv_mv, cc_pct);
}

struct guard_limit_case {
	int band;
	int tier;
	u32 fv_mv;
	u32 cc_ua;
};

static void guard_test_limit_cases(struct kunit *test,
				   const struct guard_table *tbl,
				   u32 design_uah, u32 fv_max_mv, u32 cc_max_ua,
				   const struct guard_limit_case *c, int n)
{
	u32 fv, cc;
	int i;

	for (i = 0; i < n; i++) {
		guard_limits(tbl, c[i].band, c[i].tier, design_uah, fv_max_mv,
			     cc_max_ua, &fv, &cc);
		KUNIT_EXPECT_EQ_MSG(test, fv, c[i].fv_mv, "band %d tier %d",
				    c[i].band, c[i].tier);
		KUNIT_EXPECT_EQ_MSG(test, cc, c[i].cc_ua, "band %d tier %d",
				    c[i].band, c[i].tier);
	}
}

/* A 4926 mAh battery whose description allows more than the table asks */
static void guard_test_limits_default(struct kunit *test)
{
	static const struct guard_limit_case c[] = {
		{ 0, 0, 4300, 1477800 }, { 0, 1, 4300,  492600 },
		{ 1, 0, 4450, 2463000 }, { 1, 1, 4450, 1477800 },
		{ 1, 2, 4450, 1477800 },
		{ 2, 0, 4450, 4926000 }, { 2, 1, 4450, 3448200 },
		{ 2, 2, 4450, 2463000 },
		{ 3, 0, 4450, 3940800 }, { 3, 1, 4450, 2463000 },
		{ 3, 2, 4450, 2463000 },
		{ 4, 0, 4300, 2463000 }, { 4, 1, 4300, 2463000 },
		{ 5, 0, 4200, 1477800 },
	};

	guard_test_limit_cases(test, &guard_table_default, 4926000, 4500,
			       6000000, c, ARRAY_SIZE(c));
}

/* The 4950 mAh battery, again with nothing capped by its description */
static void guard_test_limits_husky(struct kunit *test)
{
	static const struct guard_limit_case c[] = {
		{ 0, 0, 4300, 2475000 }, { 0, 1, 4300, 1485000 },
		{ 1, 0, 4480, 3465000 }, { 1, 1, 4480, 1485000 },
		{ 1, 2, 4480, 1485000 }, { 1, 3, 4480, 1485000 },
		{ 2, 0, 4480, 5940000 }, { 2, 1, 4480, 5940000 },
		{ 2, 2, 4480, 5940000 }, { 2, 3, 4480, 2475000 },
		{ 3, 0, 4480, 4950000 }, { 3, 1, 4480, 4950000 },
		{ 3, 2, 4480, 4950000 }, { 3, 3, 4480, 2475000 },
		{ 4, 0, 4300, 2475000 }, { 4, 1, 4300, 2475000 },
		{ 5, 0, 4200, 1485000 },
	};

	guard_test_limit_cases(test, &guard_table_husky, 4950000, 4500,
			       6000000, c, ARRAY_SIZE(c));
}

/* The limits from the battery description cap whatever the table says */
static void guard_test_limits_capped(struct kunit *test)
{
	static const struct guard_limit_case def[] = {
		{ 0, 0, 4300, 1477800 }, { 0, 1, 4300,  492600 },
		{ 2, 0, 4350, 2000000 }, { 2, 1, 4350, 2000000 },
		{ 2, 2, 4350, 2000000 }, { 5, 0, 4200, 1477800 },
	};
	const struct guard_table *tbl = &guard_table_husky;
	u32 fv, cc;
	int b, t;

	guard_test_limit_cases(test, &guard_table_default, 4926000, 4350,
			       2000000, def, ARRAY_SIZE(def));

	/*
	 * The husky battery node today: 4.20 V and 1.485 A, which is 30 % of
	 * 4950 mAh, the lowest row of its table. Every band and tier that
	 * charges gets exactly these two limits.
	 */
	for (b = 0; b < GUARD_NBANDS; b++) {
		for (t = 0; t <= guard_top_tier(tbl, b); t++) {
			guard_limits(tbl, b, t, 4950000, 4200, 1485000, &fv, &cc);
			KUNIT_EXPECT_EQ_MSG(test, fv, 4200, "band %d tier %d",
					    b, t);
			KUNIT_EXPECT_EQ_MSG(test, cc, 1485000, "band %d tier %d",
					    b, t);
		}
	}
}

/* The float is the step at or below the request, as stock rounds it */
static void round_test_float(struct kunit *test)
{
	u32 sel, mv;

	KUNIT_EXPECT_EQ(test, fv_selector(4200, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x14);
	KUNIT_EXPECT_EQ(test, fv_selector(4209, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x14);
	KUNIT_EXPECT_EQ(test, fv_selector(4450, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x2d);
	KUNIT_EXPECT_EQ(test, fv_selector(4500, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x32);

	/* Between the two ranges: the 3900 mV step below */
	KUNIT_EXPECT_EQ(test, fv_selector(3950, &sel), 0);
	KUNIT_EXPECT_EQ(test, linear_range_get_value_array(chg_cv_prm_ranges,
							   ARRAY_SIZE(chg_cv_prm_ranges),
							   sel, &mv), 0);
	KUNIT_EXPECT_EQ(test, mv, 3900);

	/* Out of range */
	KUNIT_EXPECT_EQ(test, fv_selector(3799, &sel), -EINVAL);
	KUNIT_EXPECT_EQ(test, fv_selector(4501, &sel), -EINVAL);
}

/*
 * The current is the step at or below the request; above the top step
 * (4000190 uA) is refused, not clamped, and so is the -EINVAL a battery
 * node without constant-charge-current-max-microamp leaves in the u32
 */
static void round_test_current(struct kunit *test)
{
	u32 sel;

	KUNIT_EXPECT_EQ(test, cc_selector(1485000, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x16);
	KUNIT_EXPECT_EQ(test, cc_selector(4000190, &sel), 0);
	KUNIT_EXPECT_EQ(test, sel, 0x3c);

	KUNIT_EXPECT_EQ(test, cc_selector(4000191, &sel), -EINVAL);
	KUNIT_EXPECT_EQ(test, cc_selector((u32)-EINVAL, &sel), -EINVAL);
	KUNIT_EXPECT_EQ(test, cc_selector(133329, &sel), -EINVAL);
}

static void stop_test_set_threshold(struct kunit *test)
{
	int end = 100, start = 0;

	/* End first, then start */
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 80, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, 70, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, end, 80);
	KUNIT_EXPECT_EQ(test, start, 70);

	/* Start first, then end: the same pair */
	end = 100;
	start = 0;
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, 70, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 80, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, end, 80);
	KUNIT_EXPECT_EQ(test, start, 70);

	/* Lowering end to 60 then start to 50, from 80 / 70 */
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 60, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, start, 0);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, 50, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, end, 60);
	KUNIT_EXPECT_EQ(test, start, 50);

	/* Raising start to 90 then end to 95, from 60 / 50 */
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, 90, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 95, &end, &start), 0);
	KUNIT_EXPECT_EQ(test, end, 95);
	KUNIT_EXPECT_EQ(test, start, 90);

	/* Out of range: nothing changes */
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 0, &end, &start), -EINVAL);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(true, 101, &end, &start), -EINVAL);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, -1, &end, &start), -EINVAL);
	KUNIT_EXPECT_EQ(test, stop_set_threshold(false, 100, &end, &start), -EINVAL);
	KUNIT_EXPECT_EQ(test, end, 95);
	KUNIT_EXPECT_EQ(test, start, 90);
}

/* 100 / 0, the default, never stops charging, whatever the charge */
static void stop_test_default_inert(struct kunit *test)
{
	int soc;

	for (soc = 0; soc <= 100; soc++) {
		KUNIT_EXPECT_FALSE_MSG(test, stop_next(false, soc, 100, 0),
				       "soc %d", soc);
		KUNIT_EXPECT_FALSE_MSG(test, stop_next(true, soc, 100, 0),
				       "soc %d", soc);
	}
}

static void stop_test_end_80(struct kunit *test)
{
	/* Start 0: resume at 75 */
	KUNIT_EXPECT_FALSE(test, stop_next(false, 79, 80, 0));
	KUNIT_EXPECT_TRUE(test, stop_next(false, 80, 80, 0));
	KUNIT_EXPECT_TRUE(test, stop_next(true, 79, 80, 0));
	KUNIT_EXPECT_TRUE(test, stop_next(true, 76, 80, 0));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 75, 80, 0));
	KUNIT_EXPECT_FALSE(test, stop_next(false, 76, 80, 0));

	/* Start 70: resume at 70 */
	KUNIT_EXPECT_TRUE(test, stop_next(true, 71, 80, 70));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 70, 80, 70));

	/* Start one below the end: resume at 79 */
	KUNIT_EXPECT_TRUE(test, stop_next(false, 80, 80, 79));
	KUNIT_EXPECT_TRUE(test, stop_next(true, 80, 80, 79));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 79, 80, 79));

	/* A start not below the end resumes at end - 5 */
	KUNIT_EXPECT_TRUE(test, stop_next(true, 76, 80, 90));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 75, 80, 90));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 75, 80, 80));

	/* An end below the gap resumes at 0 */
	KUNIT_EXPECT_TRUE(test, stop_next(true, 1, 3, 0));
	KUNIT_EXPECT_FALSE(test, stop_next(true, 0, 3, 0));
}

/* A charger regmap backed by an array, logging the writes */
struct fake_chip {
	u8 regs[256];
	u8 log[32];
	int nlog;
	int nread;
	int corrupt_reg;	/* reads of this register come back wrong */
	int fail_writes;	/* this many writes fail with -EIO first */
};

static int fake_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct fake_chip *c = ctx;

	c->nread++;
	*val = c->regs[reg];
	if (reg == c->corrupt_reg)
		*val ^= 0x01;
	return 0;
}

static int fake_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct fake_chip *c = ctx;

	if (c->fail_writes) {
		c->fail_writes--;
		return -EIO;
	}
	c->regs[reg] = val;
	if (c->nlog < ARRAY_SIZE(c->log))
		c->log[c->nlog++] = reg;
	return 0;
}

static const struct regmap_config fake_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xcc,
	.reg_read = fake_read,
	.reg_write = fake_write,
	.cache_type = REGCACHE_NONE,
};

static struct max77759_charger *fake_charger(struct kunit *test,
					     struct fake_chip **chip)
{
	struct max77759_charger *chg;
	struct device *dev;

	dev = kunit_device_register(test, "max77759-chg-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	*chip = kunit_kzalloc(test, sizeof(**chip), GFP_KERNEL);
	chg = kunit_kzalloc(test, sizeof(*chg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, *chip);
	KUNIT_ASSERT_NOT_NULL(test, chg);
	(*chip)->corrupt_reg = -1;
	chg->dev = dev;
	chg->regmap = devm_regmap_init(dev, NULL, *chip, &fake_regmap_config);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, chg->regmap);
	mutex_init(&chg->lock);
	chg->end_threshold = 100;
	return chg;
}

/* Entering charge mode with a threshold stop held gives buck only */
static void inhibit_test_set_mode(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *mode = &c->regs[MAX77759_CHGR_REG_CHG_CNFG_00];

	chg->mode = MAX77759_CHGR_MODE_OFF;
	chg->stop_inhibit = true;
	KUNIT_EXPECT_EQ(test, charger_set_mode(chg, MAX77759_CHGR_MODE_CHG_BUCK_ON), 0);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
	KUNIT_EXPECT_EQ(test, chg->mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);

	/* The unbind action clears it */
	max77759_charger_clear_stop(chg);
	KUNIT_EXPECT_FALSE(test, chg->stop_inhibit);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

/* A mode write that fails is tried again at the next call */
static void inhibit_test_write_retry(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *mode = &c->regs[MAX77759_CHGR_REG_CHG_CNFG_00];

	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	*mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;

	c->fail_writes = 1;
	charger_set_inhibit(chg, &chg->stop_inhibit, true);
	KUNIT_EXPECT_FALSE(test, chg->stop_inhibit);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);

	charger_set_inhibit(chg, &chg->stop_inhibit, true);
	KUNIT_EXPECT_TRUE(test, chg->stop_inhibit);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
}

/* A fake battery power supply for stop_work() to read the capacity of */
static int fake_bat_capacity;

static int fake_bat_get(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	if (psp != POWER_SUPPLY_PROP_CAPACITY)
		return -EINVAL;
	val->intval = fake_bat_capacity;
	return 0;
}

static const enum power_supply_property fake_bat_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc fake_bat_desc = {
	.name = "max77759-test-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = fake_bat_props,
	.num_properties = ARRAY_SIZE(fake_bat_props),
	.get_property = fake_bat_get,
};

/* stop_work() stops at the end threshold and resumes at the start one */
static void stop_test_work_capacity(struct kunit *test)
{
	struct power_supply_config cfg = { };
	struct power_supply *bat;
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *mode = &c->regs[MAX77759_CHGR_REG_CHG_CNFG_00];

	bat = devm_power_supply_register(chg->dev, &fake_bat_desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, bat);
	/* Not guard_psy: the module's own driver may be reading that */
	chg->bat_psy = fake_bat_desc.name;

	chg->guard_table = &guard_table_default;
	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	*mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	chg->end_threshold = 80;
	chg->start_threshold = 70;

	fake_bat_capacity = 79;
	stop_work(chg);
	KUNIT_EXPECT_FALSE(test, chg->stop_inhibit);
	fake_bat_capacity = 80;
	stop_work(chg);
	KUNIT_EXPECT_TRUE(test, chg->stop_inhibit);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
	fake_bat_capacity = 71;
	stop_work(chg);
	KUNIT_EXPECT_TRUE(test, chg->stop_inhibit);
	fake_bat_capacity = 70;
	stop_work(chg);
	KUNIT_EXPECT_FALSE(test, chg->stop_inhibit);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

/* Either inhibit puts the chip in buck-only mode; both must clear */
static void inhibit_test_two_flags(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *mode = &c->regs[MAX77759_CHGR_REG_CHG_CNFG_00];

	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	*mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;

	guard_set_inhibit(chg, true);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
	charger_set_inhibit(chg, &chg->stop_inhibit, true);
	guard_set_inhibit(chg, false);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
	charger_set_inhibit(chg, &chg->stop_inhibit, false);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);
	/* One write to inhibit, one to resume */
	KUNIT_EXPECT_EQ(test, c->nlog, 2);
}

/* At 100 / 0: no i2c at all */
static void stop_test_inert_no_io(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);

	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	chg->guard_table = &guard_table_default;

	stop_work(chg);

	KUNIT_EXPECT_EQ(test, c->nread, 0);
	KUNIT_EXPECT_EQ(test, c->nlog, 0);
	KUNIT_EXPECT_FALSE(test, chg->stop_inhibit);
}

/*
 * Buck only with the cable in and an inhibit held reads Not charging; with
 * the cable out it reads Discharging, inhibit or not.
 */
static void inhibit_test_status(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *int_ok = &c->regs[MAX77759_CHGR_REG_CHG_INT_OK];

	c->regs[MAX77759_CHGR_REG_CHG_DETAILS_01] =
		FIELD_PREP(MAX77759_CHGR_REG_CHG_DETAILS_01_CHG_DTLS,
			   MAX77759_CHGR_CHG_DTLS_OFF);
	*int_ok = MAX77759_CHGR_REG_CHG_INT_CHGIN;

	chg->mode = MAX77759_CHGR_MODE_OFF;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_DISCHARGING);

	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_DISCHARGING);
	chg->stop_inhibit = true;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_NOT_CHARGING);
	chg->stop_inhibit = false;
	chg->inhibit = true;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_NOT_CHARGING);
}

/* An inhibit held with no input: the battery feeds the phone */
static void inhibit_test_status_unplugged(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);

	c->regs[MAX77759_CHGR_REG_CHG_DETAILS_01] =
		FIELD_PREP(MAX77759_CHGR_REG_CHG_DETAILS_01_CHG_DTLS,
			   MAX77759_CHGR_CHG_DTLS_OFF);
	c->regs[MAX77759_CHGR_REG_CHG_INT_OK] = 0;

	/* The mode stays as it was: the psy work may not have run */
	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	chg->stop_inhibit = true;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_DISCHARGING);
	chg->stop_inhibit = false;
	chg->inhibit = true;
	KUNIT_EXPECT_EQ(test, get_status(chg), POWER_SUPPLY_STATUS_DISCHARGING);
}

static int chg_prop_get(struct power_supply *psy,
			enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = power_supply_get_property(psy, psp, &val);

	return ret ? ret : val.intval;
}

static int chg_prop_set(struct power_supply *psy,
			enum power_supply_property psp, int v)
{
	union power_supply_propval val = { .intval = v };

	return power_supply_set_property(psy, psp, &val);
}

/* charge_control_start_threshold reads the value in force */
static void stop_test_start_readback(struct kunit *test)
{
	const enum power_supply_property end =
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
	const enum power_supply_property start =
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD;
	struct power_supply_config cfg = { };
	struct power_supply_desc *desc;
	struct power_supply *psy;
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);

	/* The charger's own ops, under a name no real supply has */
	desc = devm_kmemdup(chg->dev, &max77759_charger_desc, sizeof(*desc),
			    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, desc);
	desc->name = "max77759-chg-test";
	cfg.drv_data = chg;
	psy = devm_power_supply_register(chg->dev, desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, psy);

	/* 100 / 0: no limit, start reads 0 */
	KUNIT_EXPECT_EQ(test, chg_prop_get(psy, start), 0);

	/* End 80, start 0: resumes at 75, and says so */
	KUNIT_ASSERT_EQ(test, chg_prop_set(psy, end, 80), 0);
	KUNIT_EXPECT_EQ(test, chg_prop_get(psy, start), 75);
	KUNIT_EXPECT_EQ(test, chg_prop_get(psy, end), 80);

	/* Start 70 */
	KUNIT_ASSERT_EQ(test, chg_prop_set(psy, start, 70), 0);
	KUNIT_EXPECT_EQ(test, chg_prop_get(psy, start), 70);

	/* A start at or above the end resumes at end - 5 */
	KUNIT_ASSERT_EQ(test, chg_prop_set(psy, start, 90), 0);
	KUNIT_EXPECT_EQ(test, chg_prop_get(psy, start), 75);
}

static void readback_test_apply(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	int hf, hc;

	/* Float first, then current, as before the read-back */
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4200, 1485000, &hf, &hc), 0);
	KUNIT_ASSERT_EQ(test, c->nlog, 2);
	KUNIT_EXPECT_EQ(test, c->log[0], MAX77759_CHGR_REG_CHG_CNFG_04);
	KUNIT_EXPECT_EQ(test, c->log[1], MAX77759_CHGR_REG_CHG_CNFG_02);
	KUNIT_EXPECT_EQ(test, c->regs[MAX77759_CHGR_REG_CHG_CNFG_04], 0x14);
	/*
	 * The current is the step at or below the request: 1485000 uA is
	 * written as 1466730 uA, selector 0x16, never above it.
	 */
	KUNIT_EXPECT_EQ(test, c->regs[MAX77759_CHGR_REG_CHG_CNFG_02], 0x16);
	KUNIT_EXPECT_EQ(test, get_fast_charge_current(chg), 1466730);

	/* An exact step stays as it is (cheetah, raven) */
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4450, 1466730, &hf, &hc), 0);
	KUNIT_EXPECT_EQ(test, get_fast_charge_current(chg), 1466730);
	/* akita's 1320000 uA */
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4450, 1320000, &hf, &hc), 0);
	KUNIT_EXPECT_EQ(test, get_fast_charge_current(chg), 1266720);

	/* A float between two steps reads back as the step below */
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4205, 1466730, &hf, &hc), 0);
	KUNIT_EXPECT_EQ(test, c->regs[MAX77759_CHGR_REG_CHG_CNFG_04], 0x14);
}

static void readback_test_mismatch(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	int hf, hc;

	/* -EIO, and what the chip holds comes back for the one dev_err */
	c->corrupt_reg = MAX77759_CHGR_REG_CHG_CNFG_04;
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4200, 1485000, &hf, &hc), -EIO);
	KUNIT_EXPECT_NE(test, hf, 4200);
	KUNIT_EXPECT_EQ(test, hc, 1466730);

	c->corrupt_reg = MAX77759_CHGR_REG_CHG_CNFG_02;
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4300, 1000000, &hf, &hc), -EIO);
	KUNIT_EXPECT_EQ(test, hf, 4300);
	KUNIT_EXPECT_NE(test, hc, 933370);

	/* A request refused before any write reads nothing back */
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4501, 1466730, &hf, &hc), -EINVAL);
	KUNIT_EXPECT_EQ(test, hf, -1);
	KUNIT_EXPECT_EQ(test, hc, -1);
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4200, 4000191, &hf, &hc), -EINVAL);
	KUNIT_EXPECT_EQ(test, charger_apply_limits(chg, 4200, (u32)-EINVAL, &hf, &hc), -EINVAL);
}

/* A limit the chip does not hold inhibits charging, a good one resumes it */
static void readback_test_inhibit(struct kunit *test)
{
	struct fake_chip *c;
	struct max77759_charger *chg = fake_charger(test, &c);
	u8 *mode = &c->regs[MAX77759_CHGR_REG_CHG_CNFG_00];

	chg->mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;
	*mode = MAX77759_CHGR_MODE_CHG_BUCK_ON;

	c->corrupt_reg = MAX77759_CHGR_REG_CHG_CNFG_04;
	KUNIT_EXPECT_EQ(test, guard_apply(chg, 4200, 1485000), -EIO);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_BUCK_ON);
	KUNIT_EXPECT_EQ(test, c->log[c->nlog - 1], MAX77759_CHGR_REG_CHG_CNFG_00);

	c->corrupt_reg = -1;
	KUNIT_EXPECT_EQ(test, guard_apply(chg, 4200, 1485000), 0);
	KUNIT_EXPECT_EQ(test, *mode, MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

static struct kunit_case guard_test_cases[] = {
	KUNIT_CASE(guard_test_band_edges),
	KUNIT_CASE(guard_test_hysteresis),
	KUNIT_CASE(guard_test_top_tier),
	KUNIT_CASE(guard_test_tier_steps_default),
	KUNIT_CASE(guard_test_tier_steps_husky),
	KUNIT_CASE(guard_test_default_table),
	KUNIT_CASE(guard_test_husky_table),
	KUNIT_CASE(guard_test_limits_default),
	KUNIT_CASE(guard_test_limits_husky),
	KUNIT_CASE(guard_test_limits_capped),
	KUNIT_CASE(round_test_float),
	KUNIT_CASE(round_test_current),
	KUNIT_CASE(stop_test_set_threshold),
	KUNIT_CASE(stop_test_default_inert),
	KUNIT_CASE(stop_test_end_80),
	KUNIT_CASE(inhibit_test_two_flags),
	KUNIT_CASE(stop_test_inert_no_io),
	KUNIT_CASE(inhibit_test_set_mode),
	KUNIT_CASE(inhibit_test_write_retry),
	KUNIT_CASE(stop_test_work_capacity),
	KUNIT_CASE(inhibit_test_status),
	KUNIT_CASE(inhibit_test_status_unplugged),
	KUNIT_CASE(stop_test_start_readback),
	KUNIT_CASE(readback_test_apply),
	KUNIT_CASE(readback_test_mismatch),
	KUNIT_CASE(readback_test_inhibit),
	{ }
};

static struct kunit_suite guard_test_suite = {
	.name = "max77759_charger_guard",
	.test_cases = guard_test_cases,
};

kunit_test_suite(guard_test_suite);
