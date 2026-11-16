// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the battery temperature guard of the MAX77759 charger
 * driver: bands, hysteresis, tiers, limits and the tables themselves.
 *
 * This file is included by max77759_charger.c and can not be built alone.
 */

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
	{ }
};

static struct kunit_suite guard_test_suite = {
	.name = "max77759_charger_guard",
	.test_cases = guard_test_cases,
};

kunit_test_suite(guard_test_suite);
