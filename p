From 12e632aeda316b9088c193a5b50692d41c288471 Mon Sep 17 00:00:00 2001
From: Jeff Boody <jboody@codeaurora.org>
Date: Fri, 19 Oct 2012 16:42:34 -0600
Subject: [PATCH] msm: kgsl: Add support for Android's sync point

The Android sync point framework will replace the synchronization
previously implemented by genlock. This change implements the KGSL
component of the sync point framework by creating a fence that is
automatically signaled by KGSL when it's timestamp expires. The
fence FD is returned to the user driver so that another process
can wait for the sync point.

Change-Id: Ifee38dfde00e551f3524f7a37833938dcdb64905
Signed-off-by: Jeff Boody <jboody@codeaurora.org>
Signed-off-by: Siddhartha Agrawal <agrawals@codeaurora.org>

msm: kgsl: implement sync compare callback

The compare callback is required to test the order that
sync pts will signal. Failing to implement the compare
callback results in kernel panics for some use cases.

Change-Id: Ibea1497c12fd8cc66087ff52d3709a07793f215f
Signed-off-by: Jeff Boody <jboody@codeaurora.org>
Signed-off-by: Siddhartha Agrawal <agrawals@codeaurora.org>
(cherry picked from commit b44e3b4fb92810703ad296a5c9f829c5f86ffe4a)
Signed-off-by: Shruthi Krishna <skrish@codeaurora.org>

msm: kgsl: Make the GPU device aware of the next pending event

The adreno core needs to know what the next event pending for
any given context is so it can mark the interupt to be fired.
If this isn't done then some timestamps that don't have a
matching waittimestamp call won't fire an interrupt. This is
dangerous on the last interrupt/event before a context goes
away.

Change-Id: Ic0dedbad71f6de07b43b0656128c76509326d645
Signed-off-by: Jordan Crouse <jcrouse@codeaurora.org>
---
 drivers/gpu/msm/Makefile                  |   1 +
 drivers/gpu/msm/adreno.c                  | 203 +++++++++++++++++++---------
 drivers/gpu/msm/adreno.h                  |  10 +-
 drivers/gpu/msm/adreno_a2xx.c             |   1 -
 drivers/gpu/msm/adreno_a3xx.c             |   3 -
 drivers/gpu/msm/adreno_a3xx_snapshot.c    |   6 +-
 drivers/gpu/msm/adreno_drawctxt.c         |  11 +-
 drivers/gpu/msm/adreno_postmortem.c       |   2 +-
 drivers/gpu/msm/adreno_ringbuffer.c       |  66 +++++----
 drivers/gpu/msm/adreno_ringbuffer.h       |   5 +
 drivers/gpu/msm/kgsl.c                    |  83 +++++++++---
 drivers/gpu/msm/kgsl_device.h             |  20 ++-
 drivers/gpu/msm/kgsl_gpummu.c             |   5 +-
 drivers/gpu/msm/kgsl_iommu.c              |   3 +-
 drivers/gpu/msm/kgsl_mmu.c                |  20 ++-
 drivers/gpu/msm/kgsl_mmu.h                |   7 +-
 drivers/gpu/msm/kgsl_pwrctrl.c            | 133 ++++++++++++++-----
 drivers/gpu/msm/kgsl_pwrctrl.h            |  17 +--
 drivers/gpu/msm/kgsl_pwrscale.c           |   9 +-
 drivers/gpu/msm/kgsl_pwrscale.h           |   7 +-
 drivers/gpu/msm/kgsl_pwrscale_idlestats.c |   2 +-
 drivers/gpu/msm/kgsl_pwrscale_msm.c       |  31 +++--
 drivers/gpu/msm/kgsl_pwrscale_trustzone.c |   7 +-
 drivers/gpu/msm/kgsl_sharedmem.c          |  15 +++
 drivers/gpu/msm/kgsl_sharedmem.h          |   2 +
 drivers/gpu/msm/kgsl_sync.c               | 214 ++++++++++++++++++++++++++++++
 drivers/gpu/msm/kgsl_sync.h               |  75 +++++++++++
 drivers/gpu/msm/z180.c                    |   9 +-
 drivers/gpu/msm/z180.h                    |   3 +
 include/linux/msm_kgsl.h                  |  18 ++-
 30 files changed, 779 insertions(+), 209 deletions(-)
 create mode 100644 drivers/gpu/msm/kgsl_sync.c
 create mode 100644 drivers/gpu/msm/kgsl_sync.h

diff --git a/drivers/gpu/msm/Makefile b/drivers/gpu/msm/Makefile
index 6cdb5f1..c7dfff6 100644
--- a/drivers/gpu/msm/Makefile
+++ b/drivers/gpu/msm/Makefile
@@ -17,6 +17,7 @@ msm_kgsl_core-$(CONFIG_MSM_KGSL_DRM) += kgsl_drm.o
 msm_kgsl_core-$(CONFIG_MSM_SCM) += kgsl_pwrscale_trustzone.o
 msm_kgsl_core-$(CONFIG_MSM_SLEEP_STATS_DEVICE) += kgsl_pwrscale_idlestats.o
 msm_kgsl_core-$(CONFIG_MSM_DCVS) += kgsl_pwrscale_msm.o
+msm_kgsl_core-$(CONFIG_SYNC) += kgsl_sync.o
 
 msm_adreno-y += \
 	adreno_ringbuffer.o \
diff --git a/drivers/gpu/msm/adreno.c b/drivers/gpu/msm/adreno.c
index 8c68e76..d4b7ba1 100644
--- a/drivers/gpu/msm/adreno.c
+++ b/drivers/gpu/msm/adreno.c
@@ -107,7 +107,7 @@ static struct adreno_device device_3d0 = {
 	.gmem_size = SZ_256K,
 	.pfp_fw = NULL,
 	.pm4_fw = NULL,
-	.wait_timeout = 10000, /* in milliseconds */
+	.wait_timeout = 0, /* in milliseconds, 0 means disabled */
 	.ib_check_level = 0,
 };
 
@@ -398,13 +398,6 @@ static void adreno_iommu_setstate(struct kgsl_device *device,
 		*cmds++ = cp_type3_packet(CP_INVALIDATE_STATE, 1);
 		*cmds++ = 0x7fff;
 		sizedwords += 2;
-		/*
-		 * add an interrupt at the end of commands so that the smmu
-		 * disable clock off function will get called
-		 */
-		*cmds++ = cp_type3_packet(CP_INTERRUPT, 1);
-		*cmds++ = CP_INT_CNTL__RB_INT_MASK;
-		sizedwords += 2;
 		/* This returns the per context timestamp but we need to
 		 * use the global timestamp for iommu clock disablement */
 		adreno_ringbuffer_issuecmds(device, adreno_ctx,
@@ -987,7 +980,7 @@ _adreno_recover_hang(struct kgsl_device *device,
 		 * them to pass */
 		adreno_ringbuffer_restore(rb, rec_data->bad_rb_buffer,
 					rec_data->bad_rb_size);
-		idle_ret = adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+		idle_ret = adreno_idle(device);
 		if (idle_ret) {
 			ret = adreno_stop(device);
 			if (ret) {
@@ -1030,7 +1023,7 @@ _adreno_recover_hang(struct kgsl_device *device,
 	if (ret || !rec_data->bad_rb_size) {
 		adreno_ringbuffer_restore(rb, rec_data->rb_buffer,
 				rec_data->rb_size);
-		ret = adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+		ret = adreno_idle(device);
 		if (ret) {
 			/* If we fail here we can try to invalidate another
 			 * context and try recovering again */
@@ -1312,19 +1305,57 @@ static inline void adreno_poke(struct kgsl_device *device)
 	adreno_regwrite(device, REG_CP_RB_WPTR, adreno_dev->ringbuffer.wptr);
 }
 
-/* Caller must hold the device mutex. */
-int adreno_idle(struct kgsl_device *device, unsigned int timeout)
+static int adreno_ringbuffer_drain(struct kgsl_device *device,
+	unsigned int *regs)
 {
 	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
 	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;
+	unsigned long wait;
+	unsigned long timeout = jiffies + msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
+
+	if (!(rb->flags & KGSL_FLAGS_STARTED))
+		return 0;
+
+	/*
+	 * The first time into the loop, wait for 100 msecs and kick wptr again
+	 * to ensure that the hardware has updated correctly.  After that, kick
+	 * it periodically every KGSL_TIMEOUT_PART msecs until the timeout
+	 * expires
+	 */
+
+	wait = jiffies + msecs_to_jiffies(100);
+
+	adreno_poke(device);
+
+	do {
+		if (time_after(jiffies, wait)) {
+			adreno_poke(device);
+
+			/* Check to see if the core is hung */
+			if (adreno_hang_detect(device, regs))
+				return -ETIMEDOUT;
+
+			wait = jiffies + msecs_to_jiffies(KGSL_TIMEOUT_PART);
+		}
+		GSL_RB_GET_READPTR(rb, &rb->rptr);
+
+		if (time_after(jiffies, timeout)) {
+			KGSL_DRV_ERR(device, "rptr: %x, wptr: %x\n",
+				rb->rptr, rb->wptr);
+			return -ETIMEDOUT;
+		}
+	} while (rb->rptr != rb->wptr);
+
+	return 0;
+}
+
+/* Caller must hold the device mutex. */
+int adreno_idle(struct kgsl_device *device)
+{
+	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
 	unsigned int rbbm_status;
-	unsigned long wait_timeout =
-		msecs_to_jiffies(adreno_dev->wait_timeout);
 	unsigned long wait_time;
 	unsigned long wait_time_part;
-	unsigned int msecs;
-	unsigned int msecs_first;
-	unsigned int msecs_part = KGSL_TIMEOUT_PART;
 	unsigned int prev_reg_val[hang_detect_regs_count];
 
 	memset(prev_reg_val, 0, sizeof(prev_reg_val));
@@ -1332,36 +1363,16 @@ int adreno_idle(struct kgsl_device *device, unsigned int timeout)
 	kgsl_cffdump_regpoll(device->id,
 		adreno_dev->gpudev->reg_rbbm_status << 2,
 		0x00000000, 0x80000000);
-	/* first, wait until the CP has consumed all the commands in
-	 * the ring buffer
-	 */
+
 retry:
-	if (rb->flags & KGSL_FLAGS_STARTED) {
-		msecs = adreno_dev->wait_timeout;
-		msecs_first = (msecs <= 100) ? ((msecs + 4) / 5) : 100;
-		wait_time = jiffies + wait_timeout;
-		wait_time_part = jiffies + msecs_to_jiffies(msecs_first);
-		adreno_poke(device);
-		do {
-			if (time_after(jiffies, wait_time_part)) {
-				adreno_poke(device);
-				wait_time_part = jiffies +
-					msecs_to_jiffies(msecs_part);
-				if ((adreno_hang_detect(device, prev_reg_val)))
-					goto err;
-			}
-			GSL_RB_GET_READPTR(rb, &rb->rptr);
-			if (time_after(jiffies, wait_time)) {
-				KGSL_DRV_ERR(device, "rptr: %x, wptr: %x\n",
-					rb->rptr, rb->wptr);
-				goto err;
-			}
-		} while (rb->rptr != rb->wptr);
-	}
+	/* First, wait for the ringbuffer to drain */
+	if (adreno_ringbuffer_drain(device, prev_reg_val))
+		goto err;
 
 	/* now, wait for the GPU to finish its operations */
-	wait_time = jiffies + wait_timeout;
-	wait_time_part = jiffies + msecs_to_jiffies(msecs_part);
+	wait_time = jiffies + ADRENO_IDLE_TIMEOUT;
+	wait_time_part = jiffies + msecs_to_jiffies(KGSL_TIMEOUT_PART);
+
 	while (time_before(jiffies, wait_time)) {
 		adreno_regread(device, adreno_dev->gpudev->reg_rbbm_status,
 			&rbbm_status);
@@ -1377,7 +1388,7 @@ retry:
 		 */
 		if (time_after(jiffies, wait_time_part)) {
 				wait_time_part = jiffies +
-					msecs_to_jiffies(msecs_part);
+					msecs_to_jiffies(KGSL_TIMEOUT_PART);
 				if ((adreno_hang_detect(device, prev_reg_val)))
 					goto err;
 		}
@@ -1388,7 +1399,7 @@ err:
 	KGSL_DRV_ERR(device, "spun too long waiting for RB to idle\n");
 	if (KGSL_STATE_DUMP_AND_RECOVER != device->state &&
 		!adreno_dump_and_recover(device)) {
-		wait_time = jiffies + wait_timeout;
+		wait_time = jiffies + ADRENO_IDLE_TIMEOUT;
 		goto retry;
 	}
 	return -ETIMEDOUT;
@@ -1435,7 +1446,7 @@ static int adreno_suspend_context(struct kgsl_device *device)
 	/* switch to NULL ctxt */
 	if (adreno_dev->drawctxt_active != NULL) {
 		adreno_drawctxt_switch(adreno_dev, NULL, 0);
-		status = adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+		status = adreno_idle(device);
 	}
 
 	return status;
@@ -1563,13 +1574,73 @@ static unsigned int _get_context_id(struct kgsl_context *k_ctxt)
 	return context_id;
 }
 
+static void adreno_next_event(struct kgsl_device *device,
+		struct kgsl_event *event)
+{
+	int status;
+	unsigned int ref_ts, enableflag;
+	unsigned int context_id = _get_context_id(event->context);
+	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
+
+	status = kgsl_check_timestamp(device, event->context, event->timestamp);
+	if (!status) {
+		kgsl_sharedmem_readl(&device->memstore, &enableflag,
+			KGSL_MEMSTORE_OFFSET(context_id, ts_cmp_enable));
+		/*
+		 * Barrier is needed here to make sure the read from memstore
+		 * has posted
+		 */
+
+		mb();
+
+		if (enableflag) {
+			kgsl_sharedmem_readl(&device->memstore, &ref_ts,
+				KGSL_MEMSTORE_OFFSET(context_id,
+					ref_wait_ts));
+
+			/* Make sure the memstore read has posted */
+			mb();
+			if (timestamp_cmp(ref_ts, event->timestamp) >= 0) {
+				kgsl_sharedmem_writel(&device->memstore,
+				KGSL_MEMSTORE_OFFSET(context_id,
+					ref_wait_ts), event->timestamp);
+				/* Make sure the memstore write is posted */
+				wmb();
+			}
+		} else {
+			unsigned int cmds[2];
+			kgsl_sharedmem_writel(&device->memstore,
+				KGSL_MEMSTORE_OFFSET(context_id,
+					ref_wait_ts), event->timestamp);
+			enableflag = 1;
+			kgsl_sharedmem_writel(&device->memstore,
+				KGSL_MEMSTORE_OFFSET(context_id,
+					ts_cmp_enable), enableflag);
+
+			/* Make sure the memstore write gets posted */
+			wmb();
+
+			/*
+			 * submit a dummy packet so that even if all
+			 * commands upto timestamp get executed we will still
+			 * get an interrupt
+			 */
+			cmds[0] = cp_type3_packet(CP_NOP, 1);
+			cmds[1] = 0;
+
+			if (adreno_dev->drawctxt_active)
+				adreno_ringbuffer_issuecmds_intr(device,
+						event->context, &cmds[0], 2);
+		}
+	}
+}
+
 static int kgsl_check_interrupt_timestamp(struct kgsl_device *device,
 		struct kgsl_context *context, unsigned int timestamp)
 {
 	int status;
 	unsigned int ref_ts, enableflag;
 	unsigned int context_id;
-	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
 
 	mutex_lock(&device->mutex);
 	context_id = _get_context_id(context);
@@ -1616,9 +1687,9 @@ static int kgsl_check_interrupt_timestamp(struct kgsl_device *device,
 			cmds[0] = cp_type3_packet(CP_NOP, 1);
 			cmds[1] = 0;
 
-			adreno_ringbuffer_issuecmds(device,
-					adreno_dev->drawctxt_active,
-					KGSL_CMD_FLAGS_NONE, &cmds[0], 2);
+			if (context)
+				adreno_ringbuffer_issuecmds_intr(device,
+						context, &cmds[0], 2);
 		}
 	}
 unlock:
@@ -1681,12 +1752,11 @@ static int adreno_waittimestamp(struct kgsl_device *device,
 	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
 	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
 	int retries = 0;
-	unsigned int msecs_first;
-	unsigned int msecs_part = KGSL_TIMEOUT_PART;
 	unsigned int ts_issued;
 	unsigned int context_id = _get_context_id(context);
 	unsigned int time_elapsed = 0;
 	unsigned int prev_reg_val[hang_detect_regs_count];
+	unsigned int wait;
 
 	memset(prev_reg_val, 0, sizeof(prev_reg_val));
 
@@ -1704,11 +1774,18 @@ static int adreno_waittimestamp(struct kgsl_device *device,
 		goto done;
 	}
 
-	/* Keep the first timeout as 100msecs before rewriting
-	 * the WPTR. Less visible impact if the WPTR has not
-	 * been updated properly.
+	/*
+	 * Make the first timeout interval 100 msecs and then try to kick the
+	 * wptr again.  This helps to ensure the wptr is updated properly.  If
+	 * the requested timeout is less than 100 msecs, then wait 20msecs which
+	 * is the minimum amount of time we can safely wait at 100HZ
 	 */
-	msecs_first = (msecs <= 100) ? ((msecs + 4) / 5) : 100;
+
+	if (msecs == 0 || msecs >= 100)
+		wait = 100;
+	else
+		wait = 20;
+
 	do {
 		/*
 		 * If the context ID is invalid, we are in a race with
@@ -1747,8 +1824,8 @@ static int adreno_waittimestamp(struct kgsl_device *device,
 				device->wait_queue,
 				kgsl_check_interrupt_timestamp(device,
 					context, timestamp),
-				msecs_to_jiffies(retries ?
-					msecs_part : msecs_first), io);
+				msecs_to_jiffies(wait), io);
+
 		mutex_lock(&device->mutex);
 
 		if (status > 0) {
@@ -1761,11 +1838,12 @@ static int adreno_waittimestamp(struct kgsl_device *device,
 		}
 		/*this wait timed out*/
 
-		time_elapsed = time_elapsed +
-				(retries ? msecs_part : msecs_first);
+		time_elapsed += wait;
+		wait = KGSL_TIMEOUT_PART;
+
 		retries++;
 
-	} while (time_elapsed < msecs);
+	} while (!msecs || time_elapsed < msecs);
 
 hang_dump:
 	/*
@@ -1947,6 +2025,7 @@ static const struct kgsl_functable adreno_functable = {
 	.drawctxt_create = adreno_drawctxt_create,
 	.drawctxt_destroy = adreno_drawctxt_destroy,
 	.setproperty = adreno_setproperty,
+	.next_event = adreno_next_event,
 };
 
 static struct platform_device_id adreno_id_table[] = {
diff --git a/drivers/gpu/msm/adreno.h b/drivers/gpu/msm/adreno.h
index fd41500..b8bc591 100644
--- a/drivers/gpu/msm/adreno.h
+++ b/drivers/gpu/msm/adreno.h
@@ -27,7 +27,7 @@
 /* Flags to control command packet settings */
 #define KGSL_CMD_FLAGS_NONE             0x00000000
 #define KGSL_CMD_FLAGS_PMODE		0x00000001
-#define KGSL_CMD_FLAGS_NO_TS_CMP	0x00000002
+#define KGSL_CMD_FLAGS_DUMMY_INTR_CMD	0x00000002
 
 /* Command identifiers */
 #define KGSL_CONTEXT_TO_MEM_IDENTIFIER	0x2EADBEEF
@@ -47,6 +47,12 @@
 
 #define ADRENO_NUM_CTX_SWITCH_ALLOWED_BEFORE_DRAW	50
 
+/* One cannot wait forever for the core to idle, so set an upper limit to the
+ * amount of time to wait for the core to go idle
+ */
+
+#define ADRENO_IDLE_TIMEOUT (20 * 1000)
+
 enum adreno_gpurev {
 	ADRENO_REV_UNKNOWN = 0,
 	ADRENO_REV_A200 = 200,
@@ -152,7 +158,7 @@ extern unsigned int hang_detect_regs[];
 extern const unsigned int hang_detect_regs_count;
 
 
-int adreno_idle(struct kgsl_device *device, unsigned int timeout);
+int adreno_idle(struct kgsl_device *device);
 void adreno_regread(struct kgsl_device *device, unsigned int offsetwords,
 				unsigned int *value);
 void adreno_regwrite(struct kgsl_device *device, unsigned int offsetwords,
diff --git a/drivers/gpu/msm/adreno_a2xx.c b/drivers/gpu/msm/adreno_a2xx.c
index e41696f..f55b30f 100644
--- a/drivers/gpu/msm/adreno_a2xx.c
+++ b/drivers/gpu/msm/adreno_a2xx.c
@@ -1718,7 +1718,6 @@ static void a2xx_cp_intrcallback(struct kgsl_device *device)
 			kgsl_sharedmem_writel(&rb->device->memstore,
 					KGSL_MEMSTORE_OFFSET(context_id,
 						ts_cmp_enable), 0);
-			device->last_expired_ctxt_id = context_id;
 			wmb();
 		}
 		KGSL_CMD_WARN(rb->device, "ringbuffer rb interrupt\n");
diff --git a/drivers/gpu/msm/adreno_a3xx.c b/drivers/gpu/msm/adreno_a3xx.c
index ed16a0c..8b624aa 100644
--- a/drivers/gpu/msm/adreno_a3xx.c
+++ b/drivers/gpu/msm/adreno_a3xx.c
@@ -2289,9 +2289,6 @@ static int a3xx_create_gmem_shadow(struct adreno_device *adreno_dev,
 	build_quad_vtxbuff(drawctxt, &drawctxt->context_gmem_shadow,
 		&tmp_ctx.cmd);
 
-	/* Dow we need to idle? */
-	/* adreno_idle(&adreno_dev->dev, KGSL_TIMEOUT_DEFAULT); */
-
 	tmp_ctx.cmd = build_gmem2sys_cmds(adreno_dev, drawctxt,
 		&drawctxt->context_gmem_shadow);
 	tmp_ctx.cmd = build_sys2gmem_cmds(adreno_dev, drawctxt,
diff --git a/drivers/gpu/msm/adreno_a3xx_snapshot.c b/drivers/gpu/msm/adreno_a3xx_snapshot.c
index a3bee4d..9846cc4 100644
--- a/drivers/gpu/msm/adreno_a3xx_snapshot.c
+++ b/drivers/gpu/msm/adreno_a3xx_snapshot.c
@@ -270,6 +270,9 @@ void *a3xx_snapshot(struct adreno_device *adreno_dev, void *snapshot,
 	regs.regs = (unsigned int *) a3xx_registers;
 	regs.count = a3xx_registers_count;
 
+	/* Disable Clock gating temporarily for the debug bus to work */
+	adreno_regwrite(device, A3XX_RBBM_CLOCK_CTL, 0x00);
+
 	/* Master set of (non debug) registers */
 	snapshot = kgsl_snapshot_add_section(device,
 		KGSL_SNAPSHOT_SECTION_REGS, snapshot, remain,
@@ -285,9 +288,6 @@ void *a3xx_snapshot(struct adreno_device *adreno_dev, void *snapshot,
 			remain, REG_CP_ME_CNTL, REG_CP_ME_STATUS,
 			64, 44);
 
-	/* Disable Clock gating temporarily for the debug bus to work */
-	adreno_regwrite(device, A3XX_RBBM_CLOCK_CTL, 0x00);
-
 	/* VPC memory */
 	snapshot = kgsl_snapshot_add_section(device,
 			KGSL_SNAPSHOT_SECTION_DEBUG, snapshot, remain,
diff --git a/drivers/gpu/msm/adreno_drawctxt.c b/drivers/gpu/msm/adreno_drawctxt.c
index b3c61da..bd22233 100644
--- a/drivers/gpu/msm/adreno_drawctxt.c
+++ b/drivers/gpu/msm/adreno_drawctxt.c
@@ -147,6 +147,7 @@ int adreno_drawctxt_create(struct kgsl_device *device,
 {
 	struct adreno_context *drawctxt;
 	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
+	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;
 	int ret;
 
 	drawctxt = kzalloc(sizeof(struct adreno_context), GFP_KERNEL);
@@ -157,6 +158,7 @@ int adreno_drawctxt_create(struct kgsl_device *device,
 	drawctxt->pagetable = pagetable;
 	drawctxt->bin_base_offset = 0;
 	drawctxt->id = context->id;
+	rb->timestamp[context->id] = 0;
 
 	if (flags & KGSL_CONTEXT_PREAMBLE)
 		drawctxt->flags |= CTXT_FLAGS_PREAMBLE;
@@ -174,6 +176,12 @@ int adreno_drawctxt_create(struct kgsl_device *device,
 	kgsl_sharedmem_writel(&device->memstore,
 			KGSL_MEMSTORE_OFFSET(drawctxt->id, ref_wait_ts),
 			KGSL_INIT_REFTIMESTAMP);
+	kgsl_sharedmem_writel(&device->memstore,
+			KGSL_MEMSTORE_OFFSET(drawctxt->id, ts_cmp_enable), 0);
+	kgsl_sharedmem_writel(&device->memstore,
+			KGSL_MEMSTORE_OFFSET(drawctxt->id, soptimestamp), 0);
+	kgsl_sharedmem_writel(&device->memstore,
+			KGSL_MEMSTORE_OFFSET(drawctxt->id, eoptimestamp), 0);
 
 	context->devctxt = drawctxt;
 	return 0;
@@ -216,8 +224,7 @@ void adreno_drawctxt_destroy(struct kgsl_device *device,
 		adreno_drawctxt_switch(adreno_dev, NULL, 0);
 	}
 
-	if (device->state != KGSL_STATE_HUNG)
-		adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+	adreno_idle(device);
 
 	kgsl_sharedmem_free(&drawctxt->gpustate);
 	kgsl_sharedmem_free(&drawctxt->context_gmem_shadow.gmemshadow);
diff --git a/drivers/gpu/msm/adreno_postmortem.c b/drivers/gpu/msm/adreno_postmortem.c
index 3cc4bcf..8864e85 100644
--- a/drivers/gpu/msm/adreno_postmortem.c
+++ b/drivers/gpu/msm/adreno_postmortem.c
@@ -913,7 +913,7 @@ int adreno_postmortem_dump(struct kgsl_device *device, int manual)
 		}
 
 		if (device->state == KGSL_STATE_ACTIVE)
-			kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);
+			kgsl_idle(device);
 
 	}
 	KGSL_LOG_DUMP(device, "POWER: FLAGS = %08lX | ACTIVE POWERLEVEL = %08X",
diff --git a/drivers/gpu/msm/adreno_ringbuffer.c b/drivers/gpu/msm/adreno_ringbuffer.c
index d23db12..5cbd26b 100644
--- a/drivers/gpu/msm/adreno_ringbuffer.c
+++ b/drivers/gpu/msm/adreno_ringbuffer.c
@@ -53,11 +53,9 @@ adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb, unsigned int numcmds,
 	unsigned int freecmds;
 	unsigned int *cmds;
 	uint cmds_gpu;
-	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
-	unsigned long wait_timeout = msecs_to_jiffies(adreno_dev->wait_timeout);
 	unsigned long wait_time;
+	unsigned long wait_timeout = msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
 	unsigned long wait_time_part;
-	unsigned int msecs_part = KGSL_TIMEOUT_PART;
 	unsigned int prev_reg_val[hang_detect_regs_count];
 
 	memset(prev_reg_val, 0, sizeof(prev_reg_val));
@@ -88,7 +86,7 @@ adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb, unsigned int numcmds,
 	}
 
 	wait_time = jiffies + wait_timeout;
-	wait_time_part = jiffies + msecs_to_jiffies(msecs_part);
+	wait_time_part = jiffies + msecs_to_jiffies(KGSL_TIMEOUT_PART);
 	/* wait for space in ringbuffer */
 	while (1) {
 		GSL_RB_GET_READPTR(rb, &rb->rptr);
@@ -102,7 +100,7 @@ adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb, unsigned int numcmds,
 		 */
 		if (time_after(jiffies, wait_time_part)) {
 			wait_time_part = jiffies +
-				msecs_to_jiffies(msecs_part);
+				msecs_to_jiffies(KGSL_TIMEOUT_PART);
 			if ((adreno_hang_detect(rb->device,
 						prev_reg_val))){
 				KGSL_DRV_ERR(rb->device,
@@ -394,7 +392,7 @@ int adreno_ringbuffer_start(struct adreno_ringbuffer *rb, unsigned int init_ram)
 	adreno_dev->gpudev->rb_init(adreno_dev, rb);
 
 	/* idle device to validate ME INIT */
-	status = adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+	status = adreno_idle(device);
 
 	if (status == 0)
 		rb->flags |= KGSL_FLAGS_STARTED;
@@ -404,11 +402,8 @@ int adreno_ringbuffer_start(struct adreno_ringbuffer *rb, unsigned int init_ram)
 
 void adreno_ringbuffer_stop(struct adreno_ringbuffer *rb)
 {
-	if (rb->flags & KGSL_FLAGS_STARTED) {
-		/* ME_HALT */
-		adreno_regwrite(rb->device, REG_CP_ME_CNTL, 0x10000000);
+	if (rb->flags & KGSL_FLAGS_STARTED)
 		rb->flags &= ~KGSL_FLAGS_STARTED;
-	}
 }
 
 int adreno_ringbuffer_init(struct kgsl_device *device)
@@ -494,9 +489,9 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 	*  error checking if needed
 	*/
 	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
-	total_sizedwords += !(flags & KGSL_CMD_FLAGS_NO_TS_CMP) ? 7 : 0;
 	/* 2 dwords to store the start of command sequence */
 	total_sizedwords += 2;
+	total_sizedwords += context ? 7 : 0;
 
 	if (adreno_is_a3xx(adreno_dev))
 		total_sizedwords += 7;
@@ -514,7 +509,7 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 	ringcmds = adreno_ringbuffer_allocspace(rb, total_sizedwords);
 	/* GPU may hang during space allocation, if thats the case the current
 	 * context may have hung the GPU */
-	if (context && context->flags & CTXT_FLAGS_GPU_HANG) {
+	if (context->flags & CTXT_FLAGS_GPU_HANG) {
 		KGSL_CTXT_WARN(rb->device,
 		"Context %p caused a gpu hang. Will not accept commands for context %d\n",
 		context, context->id);
@@ -548,9 +543,10 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 
 	/* always increment the global timestamp. once. */
 	rb->timestamp[KGSL_MEMSTORE_GLOBAL]++;
-	if (context) {
+
+	if (context && !(flags & KGSL_CMD_FLAGS_DUMMY_INTR_CMD)) {
 		if (context_id == KGSL_MEMSTORE_GLOBAL)
-			rb->timestamp[context_id] =
+			rb->timestamp[context->id] =
 				rb->timestamp[KGSL_MEMSTORE_GLOBAL];
 		else
 			rb->timestamp[context_id]++;
@@ -580,7 +576,7 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 		GSL_RB_WRITE(ringcmds, rcmd_gpu,
 			cp_type3_packet(CP_MEM_WRITE, 2));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, (gpuaddr +
-			KGSL_MEMSTORE_OFFSET(context->id, soptimestamp)));
+			KGSL_MEMSTORE_OFFSET(context_id, soptimestamp)));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, timestamp);
 
 		/* end-of-pipeline timestamp */
@@ -588,14 +584,14 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 			cp_type3_packet(CP_EVENT_WRITE, 3));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, CACHE_FLUSH_TS);
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, (gpuaddr +
-			KGSL_MEMSTORE_OFFSET(context->id, eoptimestamp)));
+			KGSL_MEMSTORE_OFFSET(context_id, eoptimestamp)));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, timestamp);
 
 		GSL_RB_WRITE(ringcmds, rcmd_gpu,
 			cp_type3_packet(CP_MEM_WRITE, 2));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, (gpuaddr +
-			      KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
-				      eoptimestamp)));
+			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
+				eoptimestamp)));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu,
 			rb->timestamp[KGSL_MEMSTORE_GLOBAL]);
 	} else {
@@ -603,13 +599,11 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 			cp_type3_packet(CP_EVENT_WRITE, 3));
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, CACHE_FLUSH_TS);
 		GSL_RB_WRITE(ringcmds, rcmd_gpu, (gpuaddr +
-			      KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
-				      eoptimestamp)));
-		GSL_RB_WRITE(ringcmds, rcmd_gpu,
-			rb->timestamp[KGSL_MEMSTORE_GLOBAL]);
+			KGSL_MEMSTORE_OFFSET(context_id, eoptimestamp)));
+		GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp[context_id]);
 	}
 
-	if (!(flags & KGSL_CMD_FLAGS_NO_TS_CMP)) {
+	if (context) {
 		/* Conditional execution based on memory values */
 		GSL_RB_WRITE(ringcmds, rcmd_gpu,
 			cp_type3_packet(CP_COND_EXEC, 4));
@@ -641,6 +635,30 @@ adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
 	return timestamp;
 }
 
+void
+adreno_ringbuffer_issuecmds_intr(struct kgsl_device *device,
+						struct kgsl_context *k_ctxt,
+						unsigned int *cmds,
+						int sizedwords)
+{
+	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
+	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;
+	struct adreno_context *a_ctxt = NULL;
+
+	if (!k_ctxt)
+		return;
+
+	a_ctxt = k_ctxt->devctxt;
+
+	if (k_ctxt->id == KGSL_CONTEXT_INVALID ||
+		a_ctxt == NULL ||
+		device->state & KGSL_STATE_HUNG)
+		return;
+
+	adreno_ringbuffer_addcmds(rb, a_ctxt, KGSL_CMD_FLAGS_DUMMY_INTR_CMD,
+			cmds, sizedwords);
+}
+
 unsigned int
 adreno_ringbuffer_issuecmds(struct kgsl_device *device,
 						struct adreno_context *drawctxt,
@@ -942,7 +960,7 @@ adreno_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
 	 * this is conservative but works reliably and is ok
 	 * even for performance simulations
 	 */
-	adreno_idle(device, KGSL_TIMEOUT_DEFAULT);
+	adreno_idle(device);
 #endif
 	/* If context hung and recovered then return error so that the
 	 * application may handle it */
diff --git a/drivers/gpu/msm/adreno_ringbuffer.h b/drivers/gpu/msm/adreno_ringbuffer.h
index 4cc57c2..6c3d9b1 100644
--- a/drivers/gpu/msm/adreno_ringbuffer.h
+++ b/drivers/gpu/msm/adreno_ringbuffer.h
@@ -110,6 +110,11 @@ unsigned int adreno_ringbuffer_issuecmds(struct kgsl_device *device,
 					unsigned int *cmdaddr,
 					int sizedwords);
 
+void adreno_ringbuffer_issuecmds_intr(struct kgsl_device *device,
+					struct kgsl_context *k_ctxt,
+					unsigned int *cmdaddr,
+					int sizedwords);
+
 void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb);
 
 void kgsl_cp_intrcallback(struct kgsl_device *device);
diff --git a/drivers/gpu/msm/kgsl.c b/drivers/gpu/msm/kgsl.c
index 9fa8358..2c4ec13 100644
--- a/drivers/gpu/msm/kgsl.c
+++ b/drivers/gpu/msm/kgsl.c
@@ -35,6 +35,7 @@
 #include "kgsl_sharedmem.h"
 #include "kgsl_device.h"
 #include "kgsl_trace.h"
+#include "kgsl_sync.h"
 
 #undef MODULE_PARAM_PREFIX
 #define MODULE_PARAM_PREFIX "kgsl."
@@ -367,6 +368,12 @@ kgsl_create_context(struct kgsl_device_private *dev_priv)
 	context->id = id;
 	context->dev_priv = dev_priv;
 
+	if (kgsl_sync_timeline_create(context)) {
+		idr_remove(&dev_priv->device->context_idr, id);
+		kfree(context);
+		return NULL;
+	}
+
 	return context;
 }
 
@@ -411,6 +418,7 @@ kgsl_context_destroy(struct kref *kref)
 {
 	struct kgsl_context *context = container_of(kref, struct kgsl_context,
 						    refcount);
+	kgsl_sync_timeline_destroy(context);
 	kfree(context);
 }
 
@@ -440,7 +448,21 @@ void kgsl_timestamp_expired(struct work_struct *work)
 		kfree(event);
 	}
 
-	device->last_expired_ctxt_id = KGSL_CONTEXT_INVALID;
+	/* Send the next pending event for each context to the device */
+	if (device->ftbl->next_event) {
+		unsigned int id = KGSL_MEMSTORE_GLOBAL;
+
+		list_for_each_entry(event, &device->events, list) {
+
+			if (!event->context)
+				continue;
+
+			if (event->context->id != id) {
+				device->ftbl->next_event(device, event);
+				id = event->context->id;
+			}
+		}
+	}
 
 	mutex_unlock(&device->mutex);
 }
@@ -452,6 +474,7 @@ static void kgsl_check_idle_locked(struct kgsl_device *device)
 	    device->state == KGSL_STATE_ACTIVE &&
 		device->requested_state == KGSL_STATE_NONE) {
 		kgsl_pwrctrl_request_state(device, KGSL_STATE_NAP);
+		kgsl_pwrscale_idle(device, 1);
 		if (kgsl_pwrctrl_sleep(device) != 0)
 			mod_timer(&device->idle_timer,
 				  jiffies +
@@ -560,7 +583,7 @@ static int kgsl_suspend_device(struct kgsl_device *device, pm_message_t state)
 			break;
 		case KGSL_STATE_ACTIVE:
 			/* Wait for the device to become idle */
-			device->ftbl->idle(device, KGSL_TIMEOUT_DEFAULT);
+			device->ftbl->idle(device);
 		case KGSL_STATE_NAP:
 		case KGSL_STATE_SLEEP:
 			/* Get the completion ready to be waited upon. */
@@ -1538,42 +1561,51 @@ static int kgsl_setup_phys_file(struct kgsl_mem_entry *entry,
 	if (ret)
 		return ret;
 
+	ret = -ERANGE;
+
 	if (phys == 0) {
-		ret = -EINVAL;
+		KGSL_CORE_ERR("kgsl_get_phys_file returned phys=0\n");
 		goto err;
 	}
 
-	if (offset >= len) {
-		ret = -EINVAL;
+	/* Make sure the length of the region, the offset and the desired
+	 * size are all page aligned or bail
+	 */
+	if ((len & ~PAGE_MASK) ||
+		(offset & ~PAGE_MASK) ||
+		(size & ~PAGE_MASK)) {
+		KGSL_CORE_ERR("length %lu, offset %u or size %u "
+				"is not page aligned\n",
+				len, offset, size);
 		goto err;
 	}
 
-	if (size == 0)
-		size = len;
-
-	/* Adjust the size of the region to account for the offset */
-	size += offset & ~PAGE_MASK;
+	/* The size or offset can never be greater than the PMEM length */
+	if (offset >= len || size > len) {
+		KGSL_CORE_ERR("offset %u or size %u "
+				"exceeds pmem length %lu\n",
+				offset, size, len);
+		goto err;
+	}
 
-	size = ALIGN(size, PAGE_SIZE);
+	/* If size is 0, then adjust it to default to the size of the region
+	 * minus the offset.  If size isn't zero, then make sure that it will
+	 * fit inside of the region.
+	 */
+	if (size == 0)
+		size = len - offset;
 
-	if (_check_region(offset & PAGE_MASK, size, len)) {
-		KGSL_CORE_ERR("Offset (%ld) + size (%d) is larger"
-			      "than pmem region length %ld\n",
-			      offset & PAGE_MASK, size, len);
-		ret = -EINVAL;
+	else if (_check_region(offset, size, len))
 		goto err;
 
-	}
-
 	entry->priv_data = filep;
 
 	entry->memdesc.pagetable = pagetable;
 	entry->memdesc.size = size;
-	entry->memdesc.physaddr = phys + (offset & PAGE_MASK);
-	entry->memdesc.hostptr = (void *) (virt + (offset & PAGE_MASK));
+	entry->memdesc.physaddr = phys + offset;
+	entry->memdesc.hostptr = (void *) (virt + offset);
 
-	ret = memdesc_sg_phys(&entry->memdesc,
-		phys + (offset & PAGE_MASK), size);
+	ret = memdesc_sg_phys(&entry->memdesc, phys + offset, size);
 	if (ret)
 		goto err;
 
@@ -2119,6 +2151,11 @@ static long kgsl_ioctl_timestamp_event(struct kgsl_device_private *dev_priv,
 			param->context_id, param->timestamp, param->priv,
 			param->len, dev_priv);
 		break;
+	case KGSL_TIMESTAMP_EVENT_FENCE:
+		ret = kgsl_add_fence_event(dev_priv->device,
+			param->context_id, param->timestamp, param->priv,
+			param->len, dev_priv);
+		break;
 	default:
 		ret = -EINVAL;
 	}
@@ -2197,6 +2234,8 @@ static long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
 		cmd = IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP;
 	else if (cmd == IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_OLD)
 		cmd = IOCTL_KGSL_CMDSTREAM_READTIMESTAMP;
+	else if (cmd == IOCTL_KGSL_TIMESTAMP_EVENT_OLD)
+		cmd = IOCTL_KGSL_TIMESTAMP_EVENT;
 
 	nr = _IOC_NR(cmd);
 
diff --git a/drivers/gpu/msm/kgsl_device.h b/drivers/gpu/msm/kgsl_device.h
index 74d6d07..fb727c9 100644
--- a/drivers/gpu/msm/kgsl_device.h
+++ b/drivers/gpu/msm/kgsl_device.h
@@ -23,6 +23,7 @@
 #include "kgsl_pwrctrl.h"
 #include "kgsl_log.h"
 #include "kgsl_pwrscale.h"
+#include <linux/sync.h>
 
 #define KGSL_TIMEOUT_NONE       0
 #define KGSL_TIMEOUT_DEFAULT    0xFFFFFFFF
@@ -58,6 +59,7 @@ struct platform_device;
 struct kgsl_device_private;
 struct kgsl_context;
 struct kgsl_power_stats;
+struct kgsl_event;
 
 struct kgsl_functable {
 	/* Mandatory functions - these functions must be implemented
@@ -68,7 +70,7 @@ struct kgsl_functable {
 		unsigned int offsetwords, unsigned int *value);
 	void (*regwrite) (struct kgsl_device *device,
 		unsigned int offsetwords, unsigned int value);
-	int (*idle) (struct kgsl_device *device, unsigned int timeout);
+	int (*idle) (struct kgsl_device *device);
 	unsigned int (*isidle) (struct kgsl_device *device);
 	int (*suspend_context) (struct kgsl_device *device);
 	int (*start) (struct kgsl_device *device, unsigned int init_ram);
@@ -111,6 +113,8 @@ struct kgsl_functable {
 	int (*setproperty) (struct kgsl_device *device,
 		enum kgsl_property_type type, void *value,
 		unsigned int sizebytes);
+	void (*next_event)(struct kgsl_device *device,
+		struct kgsl_event *event);
 };
 
 /* MH register values */
@@ -159,7 +163,6 @@ struct kgsl_device {
 	uint32_t state;
 	uint32_t requested_state;
 
-	unsigned int last_expired_ctxt_id;
 	unsigned int active_cnt;
 	struct completion suspend_gate;
 
@@ -218,8 +221,7 @@ void kgsl_timestamp_expired(struct work_struct *work);
 	.mutex = __MUTEX_INITIALIZER((_dev).mutex),\
 	.state = KGSL_STATE_INIT,\
 	.ver_major = DRIVER_VERSION_MAJOR,\
-	.ver_minor = DRIVER_VERSION_MINOR,\
-	.last_expired_ctxt_id = KGSL_CONTEXT_INVALID
+	.ver_minor = DRIVER_VERSION_MINOR
 
 struct kgsl_context {
 	struct kref refcount;
@@ -235,6 +237,12 @@ struct kgsl_context {
 	 * context was responsible for causing it
 	 */
 	unsigned int reset_status;
+
+	/*
+	 * Timeline used to create fences that can be signaled when a
+	 * sync_pt timestamp expires.
+	 */
+	struct sync_timeline *timeline;
 };
 
 struct kgsl_process_private {
@@ -286,9 +294,9 @@ static inline void kgsl_regwrite(struct kgsl_device *device,
 	device->ftbl->regwrite(device, offsetwords, value);
 }
 
-static inline int kgsl_idle(struct kgsl_device *device, unsigned int timeout)
+static inline int kgsl_idle(struct kgsl_device *device)
 {
-	return device->ftbl->idle(device, timeout);
+	return device->ftbl->idle(device);
 }
 
 static inline unsigned int kgsl_gpuid(struct kgsl_device *device,
diff --git a/drivers/gpu/msm/kgsl_gpummu.c b/drivers/gpu/msm/kgsl_gpummu.c
index c121110..5eb7f9a 100644
--- a/drivers/gpu/msm/kgsl_gpummu.c
+++ b/drivers/gpu/msm/kgsl_gpummu.c
@@ -472,7 +472,7 @@ static void kgsl_gpummu_default_setstate(struct kgsl_mmu *mmu,
 		return;
 
 	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
-		kgsl_idle(mmu->device, KGSL_TIMEOUT_DEFAULT);
+		kgsl_idle(mmu->device);
 		gpummu_pt = mmu->hwpagetable->priv;
 		kgsl_regwrite(mmu->device, MH_MMU_PT_BASE,
 			gpummu_pt->base.gpuaddr);
@@ -552,7 +552,7 @@ static int kgsl_gpummu_start(struct kgsl_mmu *mmu)
 	kgsl_regwrite(device, MH_MMU_CONFIG, mmu->config);
 
 	/* idle device */
-	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);
+	kgsl_idle(device);
 
 	/* enable axi interrupts */
 	kgsl_regwrite(device, MH_INTERRUPT_MASK,
@@ -685,7 +685,6 @@ kgsl_gpummu_map(void *mmu_specific_pt,
 
 static void kgsl_gpummu_stop(struct kgsl_mmu *mmu)
 {
-	kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000000);
 	mmu->flags &= ~KGSL_FLAGS_STARTED;
 }
 
diff --git a/drivers/gpu/msm/kgsl_iommu.c b/drivers/gpu/msm/kgsl_iommu.c
index fe4fd10..b9f512c 100644
--- a/drivers/gpu/msm/kgsl_iommu.c
+++ b/drivers/gpu/msm/kgsl_iommu.c
@@ -922,7 +922,6 @@ static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
 	 */
 
 	if (mmu->flags & KGSL_FLAGS_STARTED) {
-		kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000000);
 		/* detach iommu attachment */
 		kgsl_detach_pagetable_iommu_domain(mmu);
 		mmu->hwpagetable = NULL;
@@ -1047,7 +1046,7 @@ static void kgsl_iommu_default_setstate(struct kgsl_mmu *mmu,
 	/* Mask off the lsb of the pt base address since lsb will not change */
 	pt_base &= (KGSL_IOMMU_TTBR0_PA_MASK << KGSL_IOMMU_TTBR0_PA_SHIFT);
 	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
-		kgsl_idle(mmu->device, KGSL_TIMEOUT_DEFAULT);
+		kgsl_idle(mmu->device);
 		for (i = 0; i < iommu->unit_count; i++) {
 			/* get the lsb value which should not change when
 			 * changing ttbr0 */
diff --git a/drivers/gpu/msm/kgsl_mmu.c b/drivers/gpu/msm/kgsl_mmu.c
index caa2407..23cc1a2 100644
--- a/drivers/gpu/msm/kgsl_mmu.c
+++ b/drivers/gpu/msm/kgsl_mmu.c
@@ -23,7 +23,7 @@
 #include "kgsl_mmu.h"
 #include "kgsl_device.h"
 #include "kgsl_sharedmem.h"
-#include "adreno_postmortem.h"
+#include "adreno.h"
 
 #define KGSL_MMU_ALIGN_SHIFT    13
 #define KGSL_MMU_ALIGN_MASK     (~((1 << KGSL_MMU_ALIGN_SHIFT) - 1))
@@ -546,6 +546,12 @@ void kgsl_setstate(struct kgsl_mmu *mmu, unsigned int context_id,
 			uint32_t flags)
 {
 	struct kgsl_device *device = mmu->device;
+	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
+
+	if (!(flags & (KGSL_MMUFLAGS_TLBFLUSH | KGSL_MMUFLAGS_PTUPDATE))
+		&& !adreno_is_a2xx(adreno_dev))
+		return;
+
 	if (KGSL_MMU_TYPE_NONE == kgsl_mmu_type)
 		return;
 	else if (device->ftbl->setstate)
@@ -560,7 +566,7 @@ void kgsl_mh_start(struct kgsl_device *device)
 	struct kgsl_mh *mh = &device->mh;
 	/* force mmu off to for now*/
 	kgsl_regwrite(device, MH_MMU_CONFIG, 0);
-	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);
+	kgsl_idle(device);
 
 	/* define physical memory range accessible by the core */
 	kgsl_regwrite(device, MH_MMU_MPU_BASE, mh->mpu_base);
@@ -822,3 +828,13 @@ void kgsl_mmu_set_mmutype(char *mmutype)
 		kgsl_mmu_type = KGSL_MMU_TYPE_NONE;
 }
 EXPORT_SYMBOL(kgsl_mmu_set_mmutype);
+
+int kgsl_mmu_gpuaddr_in_range(unsigned int gpuaddr)
+{
+	if (KGSL_MMU_TYPE_NONE == kgsl_mmu_type)
+		return 1;
+	return ((gpuaddr >= KGSL_PAGETABLE_BASE) &&
+		(gpuaddr < (KGSL_PAGETABLE_BASE + kgsl_mmu_get_ptsize())));
+}
+EXPORT_SYMBOL(kgsl_mmu_gpuaddr_in_range);
+
diff --git a/drivers/gpu/msm/kgsl_mmu.h b/drivers/gpu/msm/kgsl_mmu.h
index f88700b..b180f72 100644
--- a/drivers/gpu/msm/kgsl_mmu.h
+++ b/drivers/gpu/msm/kgsl_mmu.h
@@ -205,6 +205,7 @@ int kgsl_mmu_enabled(void);
 void kgsl_mmu_set_mmutype(char *mmutype);
 enum kgsl_mmutype kgsl_mmu_get_mmutype(void);
 unsigned int kgsl_mmu_get_ptsize(void);
+int kgsl_mmu_gpuaddr_in_range(unsigned int gpuaddr);
 
 /*
  * Static inline functions of MMU that simply call the SMMU specific
@@ -301,12 +302,6 @@ static inline void kgsl_mmu_disable_clk_on_ts(struct kgsl_mmu *mmu,
 		mmu->mmu_ops->mmu_disable_clk_on_ts(mmu, ts, ts_valid);
 }
 
-static inline int kgsl_mmu_gpuaddr_in_range(unsigned int gpuaddr)
-{
-	return ((gpuaddr >= KGSL_PAGETABLE_BASE) &&
-		(gpuaddr < (KGSL_PAGETABLE_BASE + kgsl_mmu_get_ptsize())));
-}
-
 static inline unsigned int kgsl_mmu_get_int_mask(void)
 {
 	/* Dont enable gpummu interrupts, if iommu is enabled */
diff --git a/drivers/gpu/msm/kgsl_pwrctrl.c b/drivers/gpu/msm/kgsl_pwrctrl.c
index 2db3fa8..f64f468 100644
--- a/drivers/gpu/msm/kgsl_pwrctrl.c
+++ b/drivers/gpu/msm/kgsl_pwrctrl.c
@@ -14,6 +14,7 @@
 #include <linux/pm_runtime.h>
 #include <mach/msm_iomap.h>
 #include <mach/msm_bus.h>
+#include <linux/ktime.h>
 
 #include "kgsl.h"
 #include "kgsl_pwrscale.h"
@@ -57,6 +58,30 @@ struct clk_pair clks[KGSL_MAX_CLKS] = {
 	},
 };
 
+/* Update the elapsed time at a particular clock level
+ * if the device is active(on_time = true).Otherwise
+ * store it as sleep time.
+ */
+static void update_clk_statistics(struct kgsl_device *device,
+				bool on_time)
+{
+	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
+	struct kgsl_clk_stats *clkstats = &pwr->clk_stats;
+	ktime_t elapsed;
+	int elapsed_us;
+	if (clkstats->start.tv64 == 0)
+		clkstats->start = ktime_get();
+	clkstats->stop = ktime_get();
+	elapsed = ktime_sub(clkstats->stop, clkstats->start);
+	elapsed_us = ktime_to_us(elapsed);
+	clkstats->elapsed += elapsed_us;
+	if (on_time)
+		clkstats->clock_time[pwr->active_pwrlevel] += elapsed_us;
+	else
+		clkstats->clock_time[pwr->num_pwrlevels - 1] += elapsed_us;
+	clkstats->start = ktime_get();
+}
+
 void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
 				unsigned int new_level)
 {
@@ -68,6 +93,9 @@ void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
 		int diff = new_level - pwr->active_pwrlevel;
 		int d = (diff > 0) ? 1 : -1;
 		int level = pwr->active_pwrlevel;
+		/* Update the clock stats */
+		update_clk_statistics(device, true);
+		/* Finally set active level */
 		pwr->active_pwrlevel = new_level;
 		if ((test_bit(KGSL_PWRFLAGS_CLK_ON, &pwr->power_flags)) ||
 			(device->state == KGSL_STATE_NAP)) {
@@ -77,8 +105,8 @@ void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
 			 * Idle the gpu core before changing the clock freq.
 			 */
 			if (pwr->idle_needed == true)
-				device->ftbl->idle(device,
-						KGSL_TIMEOUT_DEFAULT);
+				device->ftbl->idle(device);
+
 			/* Don't shift by more than one level at a time to
 			 * avoid glitches.
 			 */
@@ -282,23 +310,51 @@ static int kgsl_pwrctrl_gpubusy_show(struct device *dev,
 {
 	int ret;
 	struct kgsl_device *device = kgsl_device_from_dev(dev);
-	struct kgsl_busy *b = &device->pwrctrl.busy;
-	ret = snprintf(buf, 17, "%7d %7d\n",
-				   b->on_time_old, b->time_old);
+	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
+	ret = snprintf(buf, PAGE_SIZE, "%7d %7d\n",
+			clkstats->on_time_old, clkstats->elapsed_old);
 	if (!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
-		b->on_time_old = 0;
-		b->time_old = 0;
+		clkstats->on_time_old = 0;
+		clkstats->elapsed_old = 0;
 	}
 	return ret;
 }
 
+static int kgsl_pwrctrl_gputop_show(struct device *dev,
+					struct device_attribute *attr,
+					char *buf)
+{
+	int ret;
+	struct kgsl_device *device = kgsl_device_from_dev(dev);
+	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
+	int i = 0;
+	char *ptr = buf;
+
+	ret = snprintf(buf, PAGE_SIZE, "%7d %7d ", clkstats->on_time_old,
+					clkstats->elapsed_old);
+	for (i = 0, ptr += ret; i < device->pwrctrl.num_pwrlevels;
+							i++, ptr += ret)
+		ret = snprintf(ptr, PAGE_SIZE, "%7d ",
+						clkstats->old_clock_time[i]);
+
+	if (!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
+		clkstats->on_time_old = 0;
+		clkstats->elapsed_old = 0;
+		for (i = 0; i < KGSL_MAX_PWRLEVELS ; i++)
+			clkstats->old_clock_time[i] = 0;
+	}
+	return (unsigned int) (ptr - buf);
+}
+
 DEVICE_ATTR(gpuclk, 0644, kgsl_pwrctrl_gpuclk_show, kgsl_pwrctrl_gpuclk_store);
 DEVICE_ATTR(max_gpuclk, 0644, kgsl_pwrctrl_max_gpuclk_show,
 	kgsl_pwrctrl_max_gpuclk_store);
 DEVICE_ATTR(pwrnap, 0664, kgsl_pwrctrl_pwrnap_show, kgsl_pwrctrl_pwrnap_store);
 DEVICE_ATTR(idle_timer, 0644, kgsl_pwrctrl_idle_timer_show,
 	kgsl_pwrctrl_idle_timer_store);
-DEVICE_ATTR(gpubusy, 0644, kgsl_pwrctrl_gpubusy_show,
+DEVICE_ATTR(gpubusy, 0444, kgsl_pwrctrl_gpubusy_show,
+	NULL);
+DEVICE_ATTR(gputop, 0444, kgsl_pwrctrl_gputop_show,
 	NULL);
 
 static const struct device_attribute *pwrctrl_attr_list[] = {
@@ -307,6 +363,7 @@ static const struct device_attribute *pwrctrl_attr_list[] = {
 	&dev_attr_pwrnap,
 	&dev_attr_idle_timer,
 	&dev_attr_gpubusy,
+	&dev_attr_gputop,
 	NULL
 };
 
@@ -320,29 +377,37 @@ void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
 	kgsl_remove_device_sysfs_files(device->dev, pwrctrl_attr_list);
 }
 
+static void update_statistics(struct kgsl_device *device)
+{
+	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
+	unsigned int on_time = 0;
+	int i;
+	int num_pwrlevels = device->pwrctrl.num_pwrlevels - 1;
+	/*PER CLK TIME*/
+	for (i = 0; i < num_pwrlevels; i++) {
+		clkstats->old_clock_time[i] = clkstats->clock_time[i];
+		on_time += clkstats->clock_time[i];
+		clkstats->clock_time[i] = 0;
+	}
+	clkstats->old_clock_time[num_pwrlevels] =
+				clkstats->clock_time[num_pwrlevels];
+	clkstats->clock_time[num_pwrlevels] = 0;
+	clkstats->on_time_old = on_time;
+	clkstats->elapsed_old = clkstats->elapsed;
+	clkstats->elapsed = 0;
+}
+
 /* Track the amount of time the gpu is on vs the total system time. *
  * Regularly update the percentage of busy time displayed by sysfs. */
 static void kgsl_pwrctrl_busy_time(struct kgsl_device *device, bool on_time)
 {
-	struct kgsl_busy *b = &device->pwrctrl.busy;
-	int elapsed;
-	if (b->start.tv_sec == 0)
-		do_gettimeofday(&(b->start));
-	do_gettimeofday(&(b->stop));
-	elapsed = (b->stop.tv_sec - b->start.tv_sec) * 1000000;
-	elapsed += b->stop.tv_usec - b->start.tv_usec;
-	b->time += elapsed;
-	if (on_time)
-		b->on_time += elapsed;
+	struct kgsl_clk_stats *clkstats = &device->pwrctrl.clk_stats;
+	update_clk_statistics(device, on_time);
 	/* Update the output regularly and reset the counters. */
-	if ((b->time > UPDATE_BUSY_VAL) ||
+	if ((clkstats->elapsed > UPDATE_BUSY_VAL) ||
 		!test_bit(KGSL_PWRFLAGS_AXI_ON, &device->pwrctrl.power_flags)) {
-		b->on_time_old = b->on_time;
-		b->time_old = b->time;
-		b->on_time = 0;
-		b->time = 0;
+		update_statistics(device);
 	}
-	do_gettimeofday(&(b->start));
 }
 
 void kgsl_pwrctrl_clk(struct kgsl_device *device, int state,
@@ -613,7 +678,7 @@ void kgsl_idle_check(struct work_struct *work)
 
 	mutex_lock(&device->mutex);
 	if (device->state & (KGSL_STATE_ACTIVE | KGSL_STATE_NAP)) {
-		kgsl_pwrscale_idle(device);
+		kgsl_pwrscale_idle(device, 0);
 
 		if (kgsl_pwrctrl_sleep(device) != 0) {
 			mod_timer(&device->idle_timer,
@@ -621,10 +686,11 @@ void kgsl_idle_check(struct work_struct *work)
 					device->pwrctrl.interval_timeout);
 			/* If the GPU has been too busy to sleep, make sure *
 			 * that is acurately reflected in the % busy numbers. */
-			device->pwrctrl.busy.no_nap_cnt++;
-			if (device->pwrctrl.busy.no_nap_cnt > UPDATE_BUSY) {
+			device->pwrctrl.clk_stats.no_nap_cnt++;
+			if (device->pwrctrl.clk_stats.no_nap_cnt >
+							 UPDATE_BUSY) {
 				kgsl_pwrctrl_busy_time(device, true);
-				device->pwrctrl.busy.no_nap_cnt = 0;
+				device->pwrctrl.clk_stats.no_nap_cnt = 0;
 			}
 		}
 	} else if (device->state & (KGSL_STATE_HUNG |
@@ -728,7 +794,7 @@ static void
 _sleep_accounting(struct kgsl_device *device)
 {
 	kgsl_pwrctrl_busy_time(device, false);
-	device->pwrctrl.busy.start.tv_sec = 0;
+	device->pwrctrl.clk_stats.start = ktime_set(0, 0);
 	device->pwrctrl.time = 0;
 	kgsl_pwrscale_sleep(device);
 }
@@ -766,6 +832,9 @@ _sleep(struct kgsl_device *device)
 				kgsl_pwrstate_to_str(device->state));
 		break;
 	}
+
+	kgsl_mmu_disable_clk_on_ts(&device->mmu, 0, false);
+
 	return 0;
 }
 
@@ -859,10 +928,10 @@ void kgsl_pwrctrl_wake(struct kgsl_device *device)
 		mod_timer(&device->idle_timer,
 				jiffies + device->pwrctrl.interval_timeout);
 		wake_lock(&device->idle_wakelock);
-		if (device->pwrctrl.restore_slumber == false)
-			pm_qos_update_request(&device->pm_qos_req_dma,
-						GPU_SWFI_LATENCY);
+		pm_qos_update_request(&device->pm_qos_req_dma,
+					GPU_SWFI_LATENCY);
 	case KGSL_STATE_ACTIVE:
+		kgsl_pwrctrl_request_state(device, KGSL_STATE_NONE);
 		break;
 	default:
 		KGSL_PWR_WARN(device, "unhandled state %s\n",
diff --git a/drivers/gpu/msm/kgsl_pwrctrl.h b/drivers/gpu/msm/kgsl_pwrctrl.h
index 501a438..a7fe2e2 100644
--- a/drivers/gpu/msm/kgsl_pwrctrl.h
+++ b/drivers/gpu/msm/kgsl_pwrctrl.h
@@ -27,14 +27,15 @@
 
 struct platform_device;
 
-struct kgsl_busy {
-	struct timeval start;
-	struct timeval stop;
-	int on_time;
-	int time;
-	int on_time_old;
-	int time_old;
+struct kgsl_clk_stats {
+	unsigned int old_clock_time[KGSL_MAX_PWRLEVELS];
+	unsigned int clock_time[KGSL_MAX_PWRLEVELS];
+	unsigned int on_time_old;
+	ktime_t start;
+	ktime_t stop;
 	unsigned int no_nap_cnt;
+	unsigned int elapsed;
+	unsigned int elapsed_old;
 };
 
 struct kgsl_pwrctrl {
@@ -56,8 +57,8 @@ struct kgsl_pwrctrl {
 	const char *regulator_name;
 	const char *irq_name;
 	s64 time;
-	struct kgsl_busy busy;
 	unsigned int restore_slumber;
+	struct kgsl_clk_stats clk_stats;
 };
 
 void kgsl_pwrctrl_irq(struct kgsl_device *device, int state);
diff --git a/drivers/gpu/msm/kgsl_pwrscale.c b/drivers/gpu/msm/kgsl_pwrscale.c
index 47b6bb2..3087b27 100644
--- a/drivers/gpu/msm/kgsl_pwrscale.c
+++ b/drivers/gpu/msm/kgsl_pwrscale.c
@@ -234,21 +234,18 @@ EXPORT_SYMBOL(kgsl_pwrscale_wake);
 void kgsl_pwrscale_busy(struct kgsl_device *device)
 {
 	if (PWRSCALE_ACTIVE(device) && device->pwrscale.policy->busy)
-		if ((!device->pwrscale.gpu_busy) &&
-			(device->requested_state != KGSL_STATE_SLUMBER))
+		if (device->requested_state != KGSL_STATE_SLUMBER)
 			device->pwrscale.policy->busy(device,
 					&device->pwrscale);
-	device->pwrscale.gpu_busy = 1;
 }
 
-void kgsl_pwrscale_idle(struct kgsl_device *device)
+void kgsl_pwrscale_idle(struct kgsl_device *device, unsigned int ignore_idle)
 {
 	if (PWRSCALE_ACTIVE(device) && device->pwrscale.policy->idle)
 		if (device->requested_state != KGSL_STATE_SLUMBER &&
 			device->requested_state != KGSL_STATE_SLEEP)
 			device->pwrscale.policy->idle(device,
-					&device->pwrscale);
-	device->pwrscale.gpu_busy = 0;
+					&device->pwrscale, ignore_idle);
 }
 EXPORT_SYMBOL(kgsl_pwrscale_idle);
 
diff --git a/drivers/gpu/msm/kgsl_pwrscale.h b/drivers/gpu/msm/kgsl_pwrscale.h
index 34698cd..ba9b1af 100644
--- a/drivers/gpu/msm/kgsl_pwrscale.h
+++ b/drivers/gpu/msm/kgsl_pwrscale.h
@@ -23,7 +23,8 @@ struct kgsl_pwrscale_policy  {
 	void (*close)(struct kgsl_device *device,
 		struct kgsl_pwrscale *pwrscale);
 	void (*idle)(struct kgsl_device *device,
-		struct kgsl_pwrscale *pwrscale);
+		struct kgsl_pwrscale *pwrscale,
+		unsigned int ignore_idle);
 	void (*busy)(struct kgsl_device *device,
 		struct kgsl_pwrscale *pwrscale);
 	void (*sleep)(struct kgsl_device *device,
@@ -36,7 +37,6 @@ struct kgsl_pwrscale {
 	struct kgsl_pwrscale_policy *policy;
 	struct kobject kobj;
 	void *priv;
-	int gpu_busy;
 	int enabled;
 };
 
@@ -64,7 +64,8 @@ int kgsl_pwrscale_attach_policy(struct kgsl_device *device,
 	struct kgsl_pwrscale_policy *policy);
 void kgsl_pwrscale_detach_policy(struct kgsl_device *device);
 
-void kgsl_pwrscale_idle(struct kgsl_device *device);
+void kgsl_pwrscale_idle(struct kgsl_device *device,
+				unsigned int ignore_idle);
 void kgsl_pwrscale_busy(struct kgsl_device *device);
 void kgsl_pwrscale_sleep(struct kgsl_device *device);
 void kgsl_pwrscale_wake(struct kgsl_device *device);
diff --git a/drivers/gpu/msm/kgsl_pwrscale_idlestats.c b/drivers/gpu/msm/kgsl_pwrscale_idlestats.c
index 4102302..fc58dd1 100644
--- a/drivers/gpu/msm/kgsl_pwrscale_idlestats.c
+++ b/drivers/gpu/msm/kgsl_pwrscale_idlestats.c
@@ -131,7 +131,7 @@ static void idlestats_busy(struct kgsl_device *device,
 }
 
 static void idlestats_idle(struct kgsl_device *device,
-			struct kgsl_pwrscale *pwrscale)
+		struct kgsl_pwrscale *pwrscale, unsigned int ignore_idle)
 {
 	int i, nr_cpu;
 	struct idlestats_priv *priv = pwrscale->priv;
diff --git a/drivers/gpu/msm/kgsl_pwrscale_msm.c b/drivers/gpu/msm/kgsl_pwrscale_msm.c
index 61d4b2d..baa0407 100644
--- a/drivers/gpu/msm/kgsl_pwrscale_msm.c
+++ b/drivers/gpu/msm/kgsl_pwrscale_msm.c
@@ -26,6 +26,7 @@ struct msm_priv {
 	struct msm_dcvs_idle idle_source;
 	struct msm_dcvs_freq freq_sink;
 	struct msm_dcvs_core_info *core_info;
+	int gpu_busy;
 };
 
 static int msm_idle_enable(struct msm_dcvs_idle *self,
@@ -89,29 +90,37 @@ static void msm_busy(struct kgsl_device *device,
 			struct kgsl_pwrscale *pwrscale)
 {
 	struct msm_priv *priv = pwrscale->priv;
-	if (priv->enabled)
+	if (priv->enabled && !priv->gpu_busy) {
 		msm_dcvs_idle(priv->handle, MSM_DCVS_IDLE_EXIT, 0);
+		priv->gpu_busy = 1;
+	}
 	return;
 }
 
 static void msm_idle(struct kgsl_device *device,
-			struct kgsl_pwrscale *pwrscale)
+		struct kgsl_pwrscale *pwrscale, unsigned int ignore_idle)
 {
 	struct msm_priv *priv = pwrscale->priv;
-	unsigned int rb_rptr, rb_wptr;
-	kgsl_regread(device, REG_CP_RB_RPTR, &rb_rptr);
-	kgsl_regread(device, REG_CP_RB_WPTR, &rb_wptr);
-
-	if (priv->enabled && (rb_rptr == rb_wptr))
-		msm_dcvs_idle(priv->handle, MSM_DCVS_IDLE_ENTER, 0);
 
+	if (priv->enabled && priv->gpu_busy)
+		if (device->ftbl->isidle(device)) {
+			msm_dcvs_idle(priv->handle, MSM_DCVS_IDLE_ENTER, 0);
+			priv->gpu_busy = 0;
+		}
 	return;
 }
 
 static void msm_sleep(struct kgsl_device *device,
 			struct kgsl_pwrscale *pwrscale)
 {
-	/* do we need to reset any parameters here? */
+	struct msm_priv *priv = pwrscale->priv;
+
+	if (priv->enabled && priv->gpu_busy) {
+		msm_dcvs_idle(priv->handle, MSM_DCVS_IDLE_ENTER, 0);
+		priv->gpu_busy = 0;
+	}
+
+	return;
 }
 
 static int msm_init(struct kgsl_device *device,
@@ -159,10 +168,10 @@ static int msm_init(struct kgsl_device *device,
 	ret = msm_dcvs_freq_sink_register(&priv->freq_sink);
 	if (ret >= 0) {
 		if (device->ftbl->isidle(device)) {
-			device->pwrscale.gpu_busy = 0;
+			priv->gpu_busy = 0;
 			msm_dcvs_idle(priv->handle, MSM_DCVS_IDLE_ENTER, 0);
 		} else {
-			device->pwrscale.gpu_busy = 1;
+			priv->gpu_busy = 1;
 		}
 		return 0;
 	}
diff --git a/drivers/gpu/msm/kgsl_pwrscale_trustzone.c b/drivers/gpu/msm/kgsl_pwrscale_trustzone.c
index 6d5f529..8f82b8e 100644
--- a/drivers/gpu/msm/kgsl_pwrscale_trustzone.c
+++ b/drivers/gpu/msm/kgsl_pwrscale_trustzone.c
@@ -118,16 +118,19 @@ static void tz_wake(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
 					device->pwrctrl.default_pwrlevel);
 }
 
-static void tz_idle(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale)
+static void tz_idle(struct kgsl_device *device, struct kgsl_pwrscale *pwrscale,
+						unsigned int ignore_idle)
 {
 	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
 	struct tz_priv *priv = pwrscale->priv;
 	struct kgsl_power_stats stats;
 	int val, idle;
 
+	if (ignore_idle)
+		return;
+
 	/* In "performance" mode the clock speed always stays
 	   the same */
-
 	if (priv->governor == TZ_GOVERNOR_PERFORMANCE)
 		return;
 
diff --git a/drivers/gpu/msm/kgsl_sharedmem.c b/drivers/gpu/msm/kgsl_sharedmem.c
index 742895e..50dfc87 100644
--- a/drivers/gpu/msm/kgsl_sharedmem.c
+++ b/drivers/gpu/msm/kgsl_sharedmem.c
@@ -436,6 +436,20 @@ static void kgsl_ebimem_free(struct kgsl_memdesc *memdesc)
 	free_contiguous_memory_by_paddr(memdesc->physaddr);
 }
 
+static int kgsl_ebimem_map_kernel(struct kgsl_memdesc *memdesc)
+{
+	if (!memdesc->hostptr) {
+		memdesc->hostptr = ioremap(memdesc->physaddr, memdesc->size);
+		if (!memdesc->hostptr) {
+			KGSL_CORE_ERR("ioremap failed, addr:0x%p, size:0x%x\n",
+				memdesc->hostptr, memdesc->size);
+			return -ENOMEM;
+		}
+	}
+
+	return 0;
+}
+
 static void kgsl_coherent_free(struct kgsl_memdesc *memdesc)
 {
 	kgsl_driver.stats.coherent -= memdesc->size;
@@ -456,6 +470,7 @@ static struct kgsl_memdesc_ops kgsl_ebimem_ops = {
 	.free = kgsl_ebimem_free,
 	.vmflags = kgsl_contiguous_vmflags,
 	.vmfault = kgsl_contiguous_vmfault,
+	.map_kernel_mem = kgsl_ebimem_map_kernel,
 };
 
 static struct kgsl_memdesc_ops kgsl_coherent_ops = {
diff --git a/drivers/gpu/msm/kgsl_sharedmem.h b/drivers/gpu/msm/kgsl_sharedmem.h
index 034ade4..7dc78b2 100644
--- a/drivers/gpu/msm/kgsl_sharedmem.h
+++ b/drivers/gpu/msm/kgsl_sharedmem.h
@@ -117,6 +117,8 @@ memdesc_sg_phys(struct kgsl_memdesc *memdesc,
 		unsigned int physaddr, unsigned int size)
 {
 	memdesc->sg = kgsl_sg_alloc(1);
+	if (!memdesc->sg)
+		return -ENOMEM;
 
 	kmemleak_not_leak(memdesc->sg);
 
diff --git a/drivers/gpu/msm/kgsl_sync.c b/drivers/gpu/msm/kgsl_sync.c
new file mode 100644
index 0000000..a2dfe3b
--- /dev/null
+++ b/drivers/gpu/msm/kgsl_sync.c
@@ -0,0 +1,214 @@
+/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
+ *
+ * This program is free software; you can redistribute it and/or modify
+ * it under the terms of the GNU General Public License version 2 and
+ * only version 2 as published by the Free Software Foundation.
+ *
+ * This program is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
+ * GNU General Public License for more details.
+ *
+ */
+
+#include <linux/file.h>
+#include <linux/slab.h>
+#include <linux/uaccess.h>
+
+#include "kgsl_sync.h"
+
+struct sync_pt *kgsl_sync_pt_create(struct sync_timeline *timeline,
+	unsigned int timestamp)
+{
+	struct sync_pt *pt;
+	pt = sync_pt_create(timeline, (int) sizeof(struct kgsl_sync_pt));
+	if (pt) {
+		struct kgsl_sync_pt *kpt = (struct kgsl_sync_pt *) pt;
+		kpt->timestamp = timestamp;
+	}
+	return pt;
+}
+
+/*
+ * This should only be called on sync_pts which have been created but
+ * not added to a fence.
+ */
+void kgsl_sync_pt_destroy(struct sync_pt *pt)
+{
+	sync_pt_free(pt);
+}
+
+static struct sync_pt *kgsl_sync_pt_dup(struct sync_pt *pt)
+{
+	struct kgsl_sync_pt *kpt = (struct kgsl_sync_pt *) pt;
+	return kgsl_sync_pt_create(pt->parent, kpt->timestamp);
+}
+
+static int kgsl_sync_pt_has_signaled(struct sync_pt *pt)
+{
+	struct kgsl_sync_pt *kpt = (struct kgsl_sync_pt *) pt;
+	struct kgsl_sync_timeline *ktimeline =
+		 (struct kgsl_sync_timeline *) pt->parent;
+	unsigned int ts = kpt->timestamp;
+	unsigned int last_ts = ktimeline->last_timestamp;
+	if (timestamp_cmp(last_ts, ts) >= 0) {
+		/* signaled */
+		return 1;
+	}
+	return 0;
+}
+
+static int kgsl_sync_pt_compare(struct sync_pt *a, struct sync_pt *b)
+{
+	struct kgsl_sync_pt *kpt_a = (struct kgsl_sync_pt *) a;
+	struct kgsl_sync_pt *kpt_b = (struct kgsl_sync_pt *) b;
+	unsigned int ts_a = kpt_a->timestamp;
+	unsigned int ts_b = kpt_b->timestamp;
+	return timestamp_cmp(ts_a, ts_b);
+}
+
+struct kgsl_fence_event_priv {
+	struct kgsl_context *context;
+};
+
+/**
+ * kgsl_fence_event_cb - Event callback for a fence timestamp event
+ * @device - The KGSL device that expired the timestamp
+ * @priv - private data for the event
+ * @context_id - the context id that goes with the timestamp
+ * @timestamp - the timestamp that triggered the event
+ *
+ * Signal a fence following the expiration of a timestamp
+ */
+
+static inline void kgsl_fence_event_cb(struct kgsl_device *device,
+	void *priv, u32 context_id, u32 timestamp)
+{
+	struct kgsl_fence_event_priv *ev = priv;
+	kgsl_sync_timeline_signal(ev->context->timeline, timestamp);
+	kgsl_context_put(ev->context);
+	kfree(ev);
+}
+
+/**
+ * kgsl_add_fence_event - Create a new fence event
+ * @device - KGSL device to create the event on
+ * @timestamp - Timestamp to trigger the event
+ * @data - Return fence fd stored in struct kgsl_timestamp_event_fence
+ * @len - length of the fence event
+ * @owner - driver instance that owns this event
+ * @returns 0 on success or error code on error
+ *
+ * Create a fence and register an event to signal the fence when
+ * the timestamp expires
+ */
+
+int kgsl_add_fence_event(struct kgsl_device *device,
+	u32 context_id, u32 timestamp, void __user *data, int len,
+	struct kgsl_device_private *owner)
+{
+	struct kgsl_fence_event_priv *event;
+	struct kgsl_timestamp_event_fence priv;
+	struct kgsl_context *context;
+	struct sync_pt *pt;
+	struct sync_fence *fence = NULL;
+	int ret = -EINVAL;
+
+	if (len != sizeof(priv))
+		return -EINVAL;
+
+	context = kgsl_find_context(owner, context_id);
+	if (context == NULL)
+		return -EINVAL;
+
+	event = kzalloc(sizeof(*event), GFP_KERNEL);
+	if (event == NULL)
+		return -ENOMEM;
+	event->context = context;
+	kgsl_context_get(context);
+
+	pt = kgsl_sync_pt_create(context->timeline, timestamp);
+	if (pt == NULL) {
+		KGSL_DRV_ERR(device, "kgsl_sync_pt_create failed\n");
+		ret = -ENOMEM;
+		goto fail_pt;
+	}
+
+	fence = sync_fence_create("kgsl-fence", pt);
+	if (fence == NULL) {
+		/* only destroy pt when not added to fence */
+		kgsl_sync_pt_destroy(pt);
+		KGSL_DRV_ERR(device, "sync_fence_create failed\n");
+		ret = -ENOMEM;
+		goto fail_fence;
+	}
+
+	priv.fence_fd = get_unused_fd_flags(0);
+	if (priv.fence_fd < 0) {
+		KGSL_DRV_ERR(device, "invalid fence fd\n");
+		ret = -EINVAL;
+		goto fail_fd;
+	}
+	sync_fence_install(fence, priv.fence_fd);
+
+	if (copy_to_user(data, &priv, sizeof(priv))) {
+		ret = -EFAULT;
+		goto fail_copy_fd;
+	}
+
+	ret = kgsl_add_event(device, context_id, timestamp,
+			kgsl_fence_event_cb, event, owner);
+	if (ret)
+		goto fail_event;
+
+	return 0;
+
+fail_event:
+fail_copy_fd:
+	/* clean up sync_fence_install */
+	sync_fence_put(fence);
+	put_unused_fd(priv.fence_fd);
+fail_fd:
+	/* clean up sync_fence_create */
+	sync_fence_put(fence);
+fail_fence:
+fail_pt:
+	kgsl_context_put(context);
+	kfree(event);
+	return ret;
+}
+
+static const struct sync_timeline_ops kgsl_sync_timeline_ops = {
+	.dup = kgsl_sync_pt_dup,
+	.has_signaled = kgsl_sync_pt_has_signaled,
+	.compare = kgsl_sync_pt_compare,
+};
+
+int kgsl_sync_timeline_create(struct kgsl_context *context)
+{
+	struct kgsl_sync_timeline *ktimeline;
+
+	context->timeline = sync_timeline_create(&kgsl_sync_timeline_ops,
+		(int) sizeof(struct kgsl_sync_timeline), "kgsl-timeline");
+	if (context->timeline == NULL)
+		return -EINVAL;
+
+	ktimeline = (struct kgsl_sync_timeline *) context->timeline;
+	ktimeline->last_timestamp = 0;
+
+	return 0;
+}
+
+void kgsl_sync_timeline_signal(struct sync_timeline *timeline,
+	unsigned int timestamp)
+{
+	struct kgsl_sync_timeline *ktimeline =
+		(struct kgsl_sync_timeline *) timeline;
+	ktimeline->last_timestamp = timestamp;
+	sync_timeline_signal(timeline);
+}
+
+void kgsl_sync_timeline_destroy(struct kgsl_context *context)
+{
+	sync_timeline_destroy(context->timeline);
+}
diff --git a/drivers/gpu/msm/kgsl_sync.h b/drivers/gpu/msm/kgsl_sync.h
new file mode 100644
index 0000000..06b3ad0
--- /dev/null
+++ b/drivers/gpu/msm/kgsl_sync.h
@@ -0,0 +1,75 @@
+/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
+ *
+ * This program is free software; you can redistribute it and/or modify
+ * it under the terms of the GNU General Public License version 2 and
+ * only version 2 as published by the Free Software Foundation.
+ *
+ * This program is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
+ * GNU General Public License for more details.
+ *
+ */
+#ifndef __KGSL_SYNC_H
+#define __KGSL_SYNC_H
+
+#include <linux/sync.h>
+#include "kgsl_device.h"
+
+struct kgsl_sync_timeline {
+	struct sync_timeline timeline;
+	unsigned int last_timestamp;
+};
+
+struct kgsl_sync_pt {
+	struct sync_pt pt;
+	unsigned int timestamp;
+};
+
+#if defined(CONFIG_SYNC)
+struct sync_pt *kgsl_sync_pt_create(struct sync_timeline *timeline,
+	unsigned int timestamp);
+void kgsl_sync_pt_destroy(struct sync_pt *pt);
+int kgsl_add_fence_event(struct kgsl_device *device,
+	u32 context_id, u32 timestamp, void __user *data, int len,
+	struct kgsl_device_private *owner);
+int kgsl_sync_timeline_create(struct kgsl_context *context);
+void kgsl_sync_timeline_signal(struct sync_timeline *timeline,
+	unsigned int timestamp);
+void kgsl_sync_timeline_destroy(struct kgsl_context *context);
+#else
+static inline struct sync_pt
+*kgsl_sync_pt_create(struct sync_timeline *timeline, unsigned int timestamp)
+{
+	return NULL;
+}
+
+static inline void kgsl_sync_pt_destroy(struct sync_pt *pt)
+{
+}
+
+static inline int kgsl_add_fence_event(struct kgsl_device *device,
+	u32 context_id, u32 timestamp, void __user *data, int len,
+	struct kgsl_device_private *owner)
+{
+	return -EINVAL;
+}
+
+static int kgsl_sync_timeline_create(struct kgsl_context *context)
+{
+	context->timeline = NULL;
+	return 0;
+}
+
+static inline void
+kgsl_sync_timeline_signal(struct sync_timeline *timeline,
+	unsigned int timestamp)
+{
+}
+
+static inline void kgsl_sync_timeline_destroy(struct kgsl_context *context)
+{
+}
+#endif
+
+#endif /* __KGSL_SYNC_H */
diff --git a/drivers/gpu/msm/z180.c b/drivers/gpu/msm/z180.c
index 693b887..0ce61b1 100644
--- a/drivers/gpu/msm/z180.c
+++ b/drivers/gpu/msm/z180.c
@@ -365,7 +365,7 @@ static int room_in_rb(struct z180_device *device)
 	return ts_diff < Z180_PACKET_COUNT;
 }
 
-static int z180_idle(struct kgsl_device *device, unsigned int timeout)
+static int z180_idle(struct kgsl_device *device)
 {
 	int status = 0;
 	struct z180_device *z180_dev = Z180_DEVICE(device);
@@ -373,7 +373,8 @@ static int z180_idle(struct kgsl_device *device, unsigned int timeout)
 	if (timestamp_cmp(z180_dev->current_timestamp,
 		z180_dev->timestamp) > 0)
 		status = z180_wait(device, NULL,
-				z180_dev->current_timestamp, timeout);
+				z180_dev->current_timestamp,
+				Z180_IDLE_TIMEOUT);
 
 	if (status)
 		KGSL_DRV_ERR(device, "z180_waittimestamp() timed out\n");
@@ -590,7 +591,7 @@ error_clk_off:
 static int z180_stop(struct kgsl_device *device)
 {
 	device->ftbl->irqctrl(device, 0);
-	z180_idle(device, KGSL_TIMEOUT_DEFAULT);
+	z180_idle(device);
 
 	del_timer_sync(&device->idle_timer);
 
@@ -858,7 +859,7 @@ z180_drawctxt_destroy(struct kgsl_device *device,
 {
 	struct z180_device *z180_dev = Z180_DEVICE(device);
 
-	z180_idle(device, KGSL_TIMEOUT_DEFAULT);
+	z180_idle(device);
 
 	if (z180_dev->ringbuffer.prevctx == context->id) {
 		z180_dev->ringbuffer.prevctx = Z180_INVALID_CONTEXT;
diff --git a/drivers/gpu/msm/z180.h b/drivers/gpu/msm/z180.h
index e5c5ef3..2962ccd 100644
--- a/drivers/gpu/msm/z180.h
+++ b/drivers/gpu/msm/z180.h
@@ -21,6 +21,9 @@
 
 #define Z180_DEFAULT_PWRSCALE_POLICY  NULL
 
+/* Wait a maximum of 10 seconds when trying to idle the core */
+#define Z180_IDLE_TIMEOUT (10 * 1000)
+
 struct z180_ringbuffer {
 	unsigned int prevctx;
 	struct kgsl_memdesc      cmdbufdesc;
diff --git a/include/linux/msm_kgsl.h b/include/linux/msm_kgsl.h
index e67190f..225983f 100644
--- a/include/linux/msm_kgsl.h
+++ b/include/linux/msm_kgsl.h
@@ -2,7 +2,7 @@
 #define _MSM_KGSL_H
 
 #define KGSL_VERSION_MAJOR        3
-#define KGSL_VERSION_MINOR        11
+#define KGSL_VERSION_MINOR        12
 
 /*context flags */
 #define KGSL_CONTEXT_SAVE_GMEM		0x00000001
@@ -478,7 +478,8 @@ struct kgsl_cff_syncmem {
 
 /*
  * A timestamp event allows the user space to register an action following an
- * expired timestamp.
+ * expired timestamp. Note IOCTL_KGSL_TIMESTAMP_EVENT has been redefined to
+ * _IOWR to support fences which need to return a fd for the priv parameter.
  */
 
 struct kgsl_timestamp_event {
@@ -489,7 +490,7 @@ struct kgsl_timestamp_event {
 	size_t len;              /* Size of the event specific blob */
 };
 
-#define IOCTL_KGSL_TIMESTAMP_EVENT \
+#define IOCTL_KGSL_TIMESTAMP_EVENT_OLD \
 	_IOW(KGSL_IOC_TYPE, 0x31, struct kgsl_timestamp_event)
 
 /* A genlock timestamp event releases an existing lock on timestamp expire */
@@ -500,6 +501,14 @@ struct kgsl_timestamp_event_genlock {
 	int handle; /* Handle of the genlock lock to release */
 };
 
+/* A fence timestamp event releases an existing lock on timestamp expire */
+
+#define KGSL_TIMESTAMP_EVENT_FENCE 2
+
+struct kgsl_timestamp_event_fence {
+	int fence_fd; /* Fence to signal */
+};
+
 /*
  * Set a property within the kernel.  Uses the same structure as
  * IOCTL_KGSL_GETPROPERTY
@@ -508,6 +517,9 @@ struct kgsl_timestamp_event_genlock {
 #define IOCTL_KGSL_SETPROPERTY \
 	_IOW(KGSL_IOC_TYPE, 0x32, struct kgsl_device_getproperty)
 
+#define IOCTL_KGSL_TIMESTAMP_EVENT \
+	_IOWR(KGSL_IOC_TYPE, 0x33, struct kgsl_timestamp_event)
+
 #ifdef __KERNEL__
 #ifdef CONFIG_MSM_KGSL_DRM
 int kgsl_gem_obj_addr(int drm_fd, int handle, unsigned long *start,
-- 
1.8.4.2
