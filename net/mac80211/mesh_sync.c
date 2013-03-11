/*
 * Copyright 2011-2012, Pavel Zubarev <pavel.zubarev@gmail.com>
 * Copyright 2011-2012, Marco Porsch <marco.porsch@s2005.tu-chemnitz.de>
 * Copyright 2011-2012, cozybit Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "ieee80211_i.h"
#include "mesh.h"
#include "driver-ops.h"

/* This is not in the standard.  It represents a tolerable tbtt drift below
 * which we do no TSF adjustment.
 */
#define TOFFSET_MINIMUM_ADJUSTMENT 10

/* This is not in the standard. It is a margin added to the
 * Toffset setpoint to mitigate TSF overcorrection
 * introduced by TSF adjustment latency.
 */
#define TOFFSET_SET_MARGIN 20

/* This is not in the standard.  It represents the maximum Toffset jump above
 * which we'll invalidate the Toffset setpoint and choose a new setpoint.  This
 * could be, for instance, in case a neighbor is restarted and its TSF counter
 * reset.
 */
#define TOFFSET_MAXIMUM_ADJUSTMENT 30000		/* 30 ms */

struct sync_method {
	u8 method;
	struct ieee80211_mesh_sync_ops ops;
};

/**
 * mesh_peer_tbtt_adjusting - check if an mp is currently adjusting its TBTT
 *
 * @ie: information elements of a management frame from the mesh peer
 */
static bool mesh_peer_tbtt_adjusting(struct ieee802_11_elems *ie)
{
	return (ie->mesh_config->meshconf_cap &
			IEEE80211_MESHCONF_CAPAB_TBTT_ADJUSTING) != 0;
}

void mesh_sync_adjust_tbtt(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	/* sdata->vif.bss_conf.beacon_int in 1024us units, 0.04% */
	u64 beacon_int_fraction = sdata->vif.bss_conf.beacon_int * 1024 / 2500;
	u64 tsf;
	u64 tsfdelta;

	spin_lock_bh(&ifmsh->sync_offset_lock);
	if (ifmsh->sync_offset_clockdrift_max < beacon_int_fraction) {
		msync_dbg(sdata, "TBTT : max clockdrift=%lld; adjusting\n",
			  (long long) ifmsh->sync_offset_clockdrift_max);
		tsfdelta = -ifmsh->sync_offset_clockdrift_max;
		ifmsh->sync_offset_clockdrift_max = 0;
	} else {
		msync_dbg(sdata, "TBTT : max clockdrift=%lld; adjusting by %llu\n",
			  (long long) ifmsh->sync_offset_clockdrift_max,
			  (unsigned long long) beacon_int_fraction);
		tsfdelta = -beacon_int_fraction;
		ifmsh->sync_offset_clockdrift_max -= beacon_int_fraction;
	}
	spin_unlock_bh(&ifmsh->sync_offset_lock);

	tsf = drv_get_tsf(local, sdata);
	if (tsf != -1ULL)
		drv_set_tsf(local, sdata, tsf + tsfdelta);
}

static void mesh_sync_offset_rx_bcn_presp(struct sta_info *sta,
					  struct ieee80211_mgmt *mgmt,
					  struct ieee802_11_elems *elems,
					  u64 t_r)
{
	struct ieee80211_sub_if_data *sdata = sta->sdata;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	u64 t_t;

	WARN_ON(ifmsh->mesh_sp_id != IEEE80211_SYNC_METHOD_NEIGHBOR_OFFSET);

	/* check offset sync conditions (13.13.2.2.1)
	 *
	 * TODO also sync to
	 * dot11MeshNbrOffsetMaxNeighbor non-peer non-MBSS neighbors
	 */

	if (elems->mesh_config && mesh_peer_tbtt_adjusting(elems)) {
		clear_sta_flag(sta, WLAN_STA_TOFFSET_KNOWN);
		msync_dbg(sdata, "STA %pM : is adjusting TBTT\n",
			  sta->sta.addr);
		return;
	}

	/* Timing offset calculation (see 13.13.2.2.2) */
	t_t = le64_to_cpu(mgmt->u.beacon.timestamp);
	sta->t_offset = t_t - t_r;

	if (test_sta_flag(sta, WLAN_STA_TOFFSET_KNOWN)) {
		s64 t_clockdrift = sta->t_offset_setpoint - sta->t_offset;
		msync_dbg(sdata,
			  "STA %pM : sta->t_offset=%lld, sta->t_offset_setpoint=%lld, t_clockdrift=%lld\n",
			  sta->sta.addr, (long long) sta->t_offset,
			  (long long) sta->t_offset_setpoint,
			  (long long) t_clockdrift);

		if (t_clockdrift > TOFFSET_MAXIMUM_ADJUSTMENT ||
		    t_clockdrift < -TOFFSET_MAXIMUM_ADJUSTMENT) {
			msync_dbg(sdata,
				  "STA %pM : t_clockdrift=%lld too large, setpoint reset\n",
				  sta->sta.addr,
				  (long long) t_clockdrift);
			clear_sta_flag(sta, WLAN_STA_TOFFSET_KNOWN);
			return;
		}

		spin_lock_bh(&ifmsh->sync_offset_lock);
		if (t_clockdrift > ifmsh->sync_offset_clockdrift_max)
			ifmsh->sync_offset_clockdrift_max = t_clockdrift;
		spin_unlock_bh(&ifmsh->sync_offset_lock);
	} else {
		sta->t_offset_setpoint = sta->t_offset - TOFFSET_SET_MARGIN;
		set_sta_flag(sta, WLAN_STA_TOFFSET_KNOWN);
		msync_dbg(sdata,
			  "STA %pM : offset was invalid, sta->t_offset=%lld\n",
			  sta->sta.addr,
			  (long long) sta->t_offset);
	}
}

static void mesh_sync_offset_adjust_tbtt(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	WARN_ON(ifmsh->mesh_sp_id != IEEE80211_SYNC_METHOD_NEIGHBOR_OFFSET);
	BUG_ON(!rcu_read_lock_held());

	spin_lock_bh(&ifmsh->sync_offset_lock);

	if (ifmsh->sync_offset_clockdrift_max > TOFFSET_MINIMUM_ADJUSTMENT) {
		/* Since ajusting the tsf here would
		 * require a possibly blocking call
		 * to the driver tsf setter, we punt
		 * the tsf adjustment to the mesh tasklet
		 */
		msync_dbg(sdata,
			  "TBTT : kicking off TBTT adjustment with clockdrift_max=%lld\n",
			  ifmsh->sync_offset_clockdrift_max);
		set_bit(MESH_WORK_DRIFT_ADJUST, &ifmsh->wrkq_flags);

		ifmsh->adjusting_tbtt = true;
	} else {
		msync_dbg(sdata,
			  "TBTT : max clockdrift=%lld; too small to adjust\n",
			  (long long)ifmsh->sync_offset_clockdrift_max);
		ifmsh->sync_offset_clockdrift_max = 0;

		ifmsh->adjusting_tbtt = false;
	}
	spin_unlock_bh(&ifmsh->sync_offset_lock);
}

static void mesh_sync_align_rx_bcn_presp(struct sta_info *sta,
					 struct ieee80211_mgmt *mgmt,
					 struct ieee802_11_elems *elems,
					 u64 t_r)
{
	sta->beacon_interval = ieee80211_tu_to_usec(le16_to_cpu(
			mgmt->u.beacon.beacon_int));

	/* Timing offset calculation (see 13.13.2.2.2) */
	sta->t_offset = le64_to_cpu(mgmt->u.beacon.timestamp) - t_r;
}

static inline int tbtt_offset_get(s64 t_offset, u32 bi_min)
{
	u32 offset;
	u64 tmp = abs64(t_offset);

	/* do_div only for unsigned values */
	offset = do_div(tmp, bi_min);

	/* invert positive offsets to get the offset towards later TBTT */
	if (t_offset >= 0)
		return  offset - bi_min;
	else
		return -offset;
}

static inline int tbtt_offset_get_local(struct ieee80211_sub_if_data *sdata, struct sta_info *sta)
{
	return tbtt_offset_get(sta->t_offset,
			       min((u32)ieee80211_tu_to_usec(
					sta->sdata->vif.bss_conf.beacon_int),
					sta->beacon_interval));
}

static inline int tbtt_offset_get_local_inv(struct sta_info *sta, struct ieee80211_sub_if_data *sdata)
{
	return tbtt_offset_get(-sta->t_offset,
			       min((u32)ieee80211_tu_to_usec(
					sta->sdata->vif.bss_conf.beacon_int),
					sta->beacon_interval));
}

static inline int tbtt_offset_get_sta(struct sta_info *staa, struct sta_info *stab)
{
	return tbtt_offset_get(stab->t_offset - staa->t_offset,
			       min(staa->beacon_interval,
				   stab->beacon_interval));
}

/*
 * get offset of STA to its next peer (or local STA) TBTT
 */
static int sta_offset_min_get(struct ieee80211_sub_if_data *sdata, struct sta_info *staa)
{
	struct sta_info *stab;
	int offset_min = INT_MIN, offset;
	struct sta_info *sta_next = NULL;

	/* STA - peer STA */
	list_for_each_entry(stab, &sdata->local->sta_list, list) {
		if (!ieee80211_vif_is_mesh(&stab->sdata->vif) ||
		    !ieee80211_sdata_running(stab->sdata) ||
		    stab->plink_state != NL80211_PLINK_ESTAB ||
		    staa == stab)
			continue;

		offset = tbtt_offset_get_sta(staa, stab);
		if (offset > offset_min) {
			offset_min = offset;
			sta_next = stab;
		}
	}

	/* STA - local STA */
	offset = tbtt_offset_get_local_inv(staa, sdata);
	if (offset > offset_min) {
		offset_min = offset;
		msync_dbg(sdata, "offset %pM - local STA %dus\n",
			  staa->sta.addr, offset_min);
	} else if (sta_next) {
		msync_dbg(sdata, "offset %pM - %pM %dus\n",
			  staa->sta.addr, sta_next->sta.addr, offset_min);
	}

	return offset_min;
}

static void mesh_sync_align_adjust_tbtt(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	struct sta_info *sta;
	int peer_offset_max = INT_MAX, local_offset_min = INT_MIN, offset;
	struct sta_info *sta_steady = NULL, *sta_next = NULL;

	WARN_ON(ifmsh->mesh_sp_id != IEEE80211_SYNC_METHOD_VENDOR);
	BUG_ON(!rcu_read_lock_held());

	/*
	 * NOTE:
	 * We only look for TBTT offsets to peer's TBTT later than ours. These
	 * are always negative (like t_offset). Thus, all following comparisons
	 * are inverse.
	 */

	list_for_each_entry(sta, &sdata->local->sta_list, list) {
		if (!ieee80211_vif_is_mesh(&sta->sdata->vif) ||
		    !ieee80211_sdata_running(sta->sdata) ||
		    sta->plink_state != NL80211_PLINK_ESTAB)
			continue;

		/* peer STA - others (including local STA) */
		offset = sta_offset_min_get(sdata, sta);
		if (offset < peer_offset_max) {
			peer_offset_max = offset;
			sta_steady = sta;
		}

		/* local STA - peer STA */
		offset = tbtt_offset_get_local(sdata, sta);
		if (offset > local_offset_min) {
			local_offset_min = offset;
			sta_next = sta;
		}
	}

	if (sta_next)
		msync_dbg(sdata, "offset local STA - %pM %dus\n",
			  sta_next->sta.addr, local_offset_min);

	if (local_offset_min < peer_offset_max) {
		msync_dbg(sdata, "I am the steady node!\n");
		ifmsh->adjusting_tbtt = false;
	} else {
		msync_dbg(sdata, "the steady node is %pM\n",
			  sta_steady->sta.addr);

		if (local_offset_min < -10000) {
			msync_dbg(sdata, "adjusting towards %pM by %dus\n",
				  sta_next->sta.addr, local_offset_min);
			/*
			 * Since adjusting the TSF here would
			 * require a possibly blocking call
			 * to the driver TSF setter, we punt
			 * the TSF adjustment to the mesh tasklet
			 */
			ifmsh->sync_offset_clockdrift_max = -local_offset_min;
			set_bit(MESH_WORK_DRIFT_ADJUST, &ifmsh->wrkq_flags);
			ifmsh->adjusting_tbtt = true;
		} else {
			ifmsh->adjusting_tbtt = false;
		}
	}
}

static const struct sync_method sync_methods[] = {
	{
		.method = IEEE80211_SYNC_METHOD_NEIGHBOR_OFFSET,
		.ops = {
			.rx_bcn_presp = &mesh_sync_offset_rx_bcn_presp,
			.adjust_tbtt = &mesh_sync_offset_adjust_tbtt,
		}
	},
	{
		.method = IEEE80211_SYNC_METHOD_VENDOR,
		.ops = {
			.rx_bcn_presp = &mesh_sync_align_rx_bcn_presp,
			.adjust_tbtt = &mesh_sync_align_adjust_tbtt,
		}
	},
};

const struct ieee80211_mesh_sync_ops *ieee80211_mesh_sync_ops_get(u8 method)
{
	int i;

	for (i = 0 ; i < ARRAY_SIZE(sync_methods); ++i) {
		if (sync_methods[i].method == method)
			return &sync_methods[i].ops;
	}
	return NULL;
}
