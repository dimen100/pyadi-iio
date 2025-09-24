# Copyright (C) 2023-2025 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

from typing import Dict, List

from adi.context_manager import context_manager
from adi.rx_tx import are_channels_complex, rx_tx
from adi.sync_start import sync_start


def _map_to_dict(paths, ch):
    """
    Align label parsing with ad9081.
    Accepts either "FDDC->CDDC->ADC" or "side:FDDC->CDDC->ADC".
    """
    if "label" not in ch.attrs:
        return paths
    lab = ch.attrs["label"].value
    if lab == "buffer_only":
        return paths

    lab = lab.replace(":", "->")
    parts = lab.split("->")
    if len(parts) == 3:
        fddc, cddc, adc = parts
    elif len(parts) == 4:
        _, fddc, cddc, adc = parts
    else:
        return paths

    if adc not in paths:
        paths[adc] = {}
    if cddc not in paths[adc]:
        paths[adc][cddc] = {}
    if fddc not in paths[adc][cddc]:
        paths[adc][cddc][fddc] = {"channels": [ch._id]}
    else:
        paths[adc][cddc][fddc]["channels"].append(ch._id)
    return paths


def _sortconv(chans_names, noq=False, dds=False, complex=False):
    """
    Same sorting semantics as ad9081: interleave I/Q when complex,
    sort by index, handle DDS altvoltage channels.
    """
    tmpI = filter(lambda k: "_i" in k, chans_names)
    tmpQ = filter(lambda k: "_q" in k, chans_names)

    assert not (dds and complex), \
        "DDS channels cannot have complex names (voltageX_i, voltageX_q)"

    def ignoreadc(w):      # voltage{N}_i/_q
        return int(w[len("voltage"): w.find("_")])

    def ignorealt(w):      # altvoltage{N}
        return int(w[len("altvoltage"):])

    def ignorevoltage(w):  # voltage{N}
        return int(w[len("voltage"):])

    out = []
    if dds:
        filt = ignorealt
        tmpI = chans_names
        noq = True
    elif not complex:
        filt = ignorevoltage
        tmpI = chans_names
        noq = True
    else:
        filt = ignoreadc

    tmpI = sorted(tmpI, key=filt)
    tmpQ = sorted(tmpQ, key=filt)
    for i in range(len(tmpI)):
        out.append(tmpI[i])
        if not noq:
            out.append(tmpQ[i])
    return out


class ad9084(rx_tx, context_manager, sync_start):
    """AD9084 Mixed-Signal Front End (MxFE) — single path (asym=0)

    Mirrors ad9081 API with:
        self._rxadc = "axi-ad9084-rx-hpc"
        self._txdac = "axi-ad9084-tx-hpc"
    """

    _complex_data = True
    _rx_channel_names: List[str] = []
    _tx_channel_names: List[str] = []
    _tx_control_channel_names: List[str] = []
    _rx_coarse_ddc_channel_names: List[str] = []
    _tx_coarse_duc_channel_names: List[str] = []
    _rx_fine_ddc_channel_names: List[str] = []
    _tx_fine_duc_channel_names: List[str] = []
    _dds_channel_names: List[str] = []
    _device_name = ""

    _path_map: Dict[str, Dict[str, Dict[str, List[str]]]] = {}

    def __init__(self, uri="",
                 rx_device_name="axi-ad9084-rx-hpc",
                 tx_device_name="axi-ad9084-tx-hpc"):

        # Reset default channel lists
        self._rx_channel_names = []
        self._tx_channel_names = []
        self._tx_control_channel_names = []
        self._rx_coarse_ddc_channel_names = []
        self._tx_coarse_duc_channel_names = []
        self._rx_fine_ddc_channel_names = []
        self._tx_fine_duc_channel_names = []
        self._dds_channel_names = []

        context_manager.__init__(self, uri, self._device_name)
        # Default device for attribute writes
        self._ctrl = self._ctx.find_device(rx_device_name)
        # Devices with buffers
        self._rxadc = self._ctx.find_device(rx_device_name)
        self._txdac = self._ctx.find_device(tx_device_name)
        if self._rxadc is None:
            raise Exception(f"No device found with name {rx_device_name}")
        if self._txdac is None:
            raise Exception(f"No device found with name {tx_device_name}")

        # Update complex data flags (match ad9081)
        if self._rx_complex_data is None:
            self._rx_complex_data = are_channels_complex(self._rxadc.channels)
        if self._tx_complex_data is None:
            self._tx_complex_data = are_channels_complex(self._txdac.channels)

        # Build DDC/DUC path map from labels
        paths = {}
        for ch in self._rxadc.channels:
            if "label" in ch.attrs:
                paths = _map_to_dict(paths, ch)
        self._path_map = paths

        # Gather data + DDS channels
        for ch in self._rxadc.channels:
            if ch.scan_element and not ch.output:
                self._rx_channel_names.append(ch._id)
        for ch in self._txdac.channels:
            if ch.scan_element:
                self._tx_channel_names.append(ch._id)
            else:
                self._dds_channel_names.append(ch._id)

        # Sort exactly like ad9081
        self._rx_channel_names = _sortconv(
            self._rx_channel_names, complex=self._rx_complex_data
        )
        self._tx_channel_names = _sortconv(
            self._tx_channel_names, complex=self._tx_complex_data
        )
        self._dds_channel_names = _sortconv(self._dds_channel_names, dds=True)

        # Map unique attributes to channel properties (same logic as ad9081)
        self._rx_fine_ddc_channel_names = []
        self._rx_coarse_ddc_channel_names = []
        self._tx_fine_duc_channel_names = []
        self._tx_coarse_duc_channel_names = []
        for converter in paths:
            for cdc in paths[converter]:
                chans = []
                for fdc in paths[converter][cdc]:
                    chans += paths[converter][cdc][fdc]["channels"]
                # keep one (I) per complex pair
                chans = [n for n in chans if "_q" not in n and "voltage" in n]
                if "ADC" in converter:
                    self._rx_coarse_ddc_channel_names.append(chans[0])
                    self._rx_fine_ddc_channel_names += chans
                else:
                    self._tx_coarse_duc_channel_names.append(chans[0])
                    self._tx_fine_duc_channel_names += chans

        rx_tx.__init__(self)
        sync_start.__init__(self)
        self.rx_buffer_size = 2 ** 16

    # ───────────────── Single-element helpers (match ad9081) ─────────────────
    def _get_iio_attr_str_single(self, channel_name, attr, output):
        if isinstance(channel_name, list):
            channel_name = channel_name[0]
        return self._get_iio_attr_str(channel_name, attr, output)

    def _set_iio_attr_str_single(self, channel_name, attr, output, value):
        if isinstance(channel_name, list):
            channel_name = channel_name[0]
        return self._set_iio_attr(channel_name, attr, output, value)

    def _get_iio_attr_single(self, channel_name, attr, output, _ctrl=None):
        if isinstance(channel_name, list):
            channel_name = channel_name[0]
        return self._get_iio_attr(channel_name, attr, output, _ctrl)

    def _set_iio_attr_single(self, channel_name, attr, output, value, _ctrl=None):
        if isinstance(channel_name, list):
            channel_name = channel_name[0]
        return self._set_iio_attr(channel_name, attr, output, value, _ctrl)

    def _get_iio_dev_attr_single(self, attr):
        return self._get_iio_dev_attr(attr)

    def _set_iio_dev_attr_single(self, attr, value):
        return self._set_iio_dev_attr(attr, value)

    def _get_iio_dev_attr_str_single(self, attr):
        return self._get_iio_dev_attr_str(attr)

    def _set_iio_dev_attr_str_single(self, attr, value):
        return self._set_iio_dev_attr_str(attr, value)

    # ───────────────────────────── Public API ─────────────────────────────

    @property
    def path_map(self):
        """Map of channelizers (coarse & fine) to driver channel names"""
        return self._path_map

    # RX fine/coarse DDC (freq/phase, test, zone, 6 dB gains)
    @property
    def rx_channel_nco_frequencies(self):
        return self._get_iio_attr_vec(
            self._rx_fine_ddc_channel_names, "channel_nco_frequency", False
        )

    @rx_channel_nco_frequencies.setter
    def rx_channel_nco_frequencies(self, value):
        self._set_iio_attr_int_vec(
            self._rx_fine_ddc_channel_names, "channel_nco_frequency", False, value
        )

    @property
    def rx_channel_nco_phases(self):
        return self._get_iio_attr_vec(
            self._rx_fine_ddc_channel_names, "channel_nco_phase", False
        )

    @rx_channel_nco_phases.setter
    def rx_channel_nco_phases(self, value):
        self._set_iio_attr_int_vec(
            self._rx_fine_ddc_channel_names, "channel_nco_phase", False, value
        )

    @property
    def rx_main_nco_frequencies(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_frequency", False
        )

    @rx_main_nco_frequencies.setter
    def rx_main_nco_frequencies(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_frequency", False, value
        )

    @property
    def rx_main_nco_phases(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_phase", False
        )

    @rx_main_nco_phases.setter
    def rx_main_nco_phases(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_phase", False, value
        )

    @property
    def rx_test_mode(self):
        return self._get_iio_attr_str_single(
            self._rx_coarse_ddc_channel_names, "test_mode", False
        )

    @rx_test_mode.setter
    def rx_test_mode(self, value):
        self._set_iio_attr_single(
            self._rx_coarse_ddc_channel_names, "test_mode", False, value
        )

    @property
    def rx_nyquist_zone(self):
        return self._get_iio_attr_str_vec(
            self._rx_coarse_ddc_channel_names, "nyquist_zone", False
        )

    @rx_nyquist_zone.setter
    def rx_nyquist_zone(self, value):
        self._set_iio_attr_str_vec(
            self._rx_coarse_ddc_channel_names, "nyquist_zone", False, value
        )

    # Optional 6 dB toggles (names aligned to ad9081)
    @property
    def rx_main_6dB_digital_gains(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_6db_digital_gain_en", False
        )

    @rx_main_6dB_digital_gains.setter
    def rx_main_6dB_digital_gains(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_6db_digital_gain_en", False, value
        )

    @property
    def rx_channel_6dB_digital_gains(self):
        return self._get_iio_attr_vec(
            self._rx_fine_ddc_channel_names, "channel_6db_digital_gain_en", False
        )

    @rx_channel_6dB_digital_gains.setter
    def rx_channel_6dB_digital_gains(self, value):
        self._set_iio_attr_int_vec(
            self._rx_fine_ddc_channel_names, "channel_6db_digital_gain_en", False, value
        )

    # RX FFH (match ad9081 names; safe no-op if attrs absent)
    @property
    def rx_main_nco_ffh_index(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_ffh_index", False
        )

    @rx_main_nco_ffh_index.setter
    def rx_main_nco_ffh_index(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_ffh_index", False, value
        )

    @property
    def rx_main_nco_ffh_select(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_ffh_select", False
        )

    @rx_main_nco_ffh_select.setter
    def rx_main_nco_ffh_select(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_nco_ffh_select", False, value
        )

    @property
    def rx_main_ffh_mode(self):
        return self._get_iio_attr_str_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_mode", False
        )

    @rx_main_ffh_mode.setter
    def rx_main_ffh_mode(self, value):
        self._set_iio_attr_str_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_mode", False, value
        )

    @property
    def rx_main_ffh_trig_hop_en(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_trig_hop_en", False
        )

    @rx_main_ffh_trig_hop_en.setter
    def rx_main_ffh_trig_hop_en(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_trig_hop_en", False, value
        )

    @property
    def rx_main_ffh_gpio_mode_enable(self):
        return self._get_iio_attr_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_gpio_mode_en", False
        )

    @rx_main_ffh_gpio_mode_enable.setter
    def rx_main_ffh_gpio_mode_enable(self, value):
        self._set_iio_attr_int_vec(
            self._rx_coarse_ddc_channel_names, "main_ffh_gpio_mode_en", False, value
        )

    # TX fine/coarse DUC (freq/phase, test, gains, FFH)
    @property
    def tx_channel_nco_frequencies(self):
        return self._get_iio_attr_vec(
            self._tx_fine_duc_channel_names, "channel_nco_frequency", True
        )

    @tx_channel_nco_frequencies.setter
    def tx_channel_nco_frequencies(self, value):
        self._set_iio_attr_int_vec(
            self._tx_fine_duc_channel_names, "channel_nco_frequency", True, value
        )

    @property
    def tx_channel_nco_phases(self):
        return self._get_iio_attr_vec(
            self._tx_fine_duc_channel_names, "channel_nco_phase", True
        )

    @tx_channel_nco_phases.setter
    def tx_channel_nco_phases(self, value):
        self._set_iio_attr_int_vec(
            self._tx_fine_duc_channel_names, "channel_nco_phase", True, value
        )

    @property
    def tx_channel_nco_test_tone_en(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_test_tone_en", True
        )

    @tx_channel_nco_test_tone_en.setter
    def tx_channel_nco_test_tone_en(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_test_tone_en", True, value
        )

    @property
    def tx_channel_nco_test_tone_scales(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_test_tone_scale", True
        )

    @tx_channel_nco_test_tone_scales.setter
    def tx_channel_nco_test_tone_scales(self, value):
        self._set_iio_attr_float_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_test_tone_scale", True, value
        )

    @property
    def tx_channel_nco_gain_scales(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_gain_scale", True
        )

    @tx_channel_nco_gain_scales.setter
    def tx_channel_nco_gain_scales(self, value):
        self._set_iio_attr_float_vec(
            self._tx_coarse_duc_channel_names, "channel_nco_gain_scale", True, value
        )

    @property
    def tx_main_nco_frequencies(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_frequency", True
        )

    @tx_main_nco_frequencies.setter
    def tx_main_nco_frequencies(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_frequency", True, value
        )

    @property
    def tx_main_nco_phases(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_phase", True
        )

    @tx_main_nco_phases.setter
    def tx_main_nco_phases(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_phase", True, value
        )

    @property
    def tx_main_nco_test_tone_en(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_test_tone_en", True
        )

    @tx_main_nco_test_tone_en.setter
    def tx_main_nco_test_tone_en(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_test_tone_en", True, value
        )

    @property
    def tx_main_nco_test_tone_scales(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_test_tone_scale", True
        )

    @tx_main_nco_test_tone_scales.setter
    def tx_main_nco_test_tone_scales(self, value):
        self._set_iio_attr_float_vec(
            self._tx_coarse_duc_channel_names, "main_nco_test_tone_scale", True, value
        )

    # TX FFH (mirror ad9081)
    @property
    def tx_main_ffh_frequency(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_frequency", True
        )

    @tx_main_ffh_frequency.setter
    def tx_main_ffh_frequency(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_frequency", True, value
        )

    @property
    def tx_main_ffh_index(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_index", True
        )

    @tx_main_ffh_index.setter
    def tx_main_ffh_index(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_index", True, value
        )

    @property
    def tx_main_nco_ffh_select(self):
        return self._get_iio_attr_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_select", True
        )

    @tx_main_nco_ffh_select.setter
    def tx_main_nco_ffh_select(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "main_nco_ffh_select", True, value
        )

    @property
    def tx_main_ffh_mode(self):
        return self._get_iio_attr_str_vec(
            self._tx_coarse_duc_channel_names, "main_ffh_mode", True
        )

    @tx_main_ffh_mode.setter
    def tx_main_ffh_mode(self, value):
        self._set_iio_attr_str_vec(
            self._tx_coarse_duc_channel_names, "main_ffh_mode", True, value
        )

    @property
    def tx_main_ffh_gpio_mode_enable(self):
        # ad9081 uses a single channel for this; mirror that API
        return self._get_iio_attr_single("voltage0_i", "main_ffh_gpio_mode_en", True)

    @tx_main_ffh_gpio_mode_enable.setter
    def tx_main_ffh_gpio_mode_enable(self, value):
        self._set_iio_attr_single(
            "voltage0_i", "main_ffh_gpio_mode_en", True, value
        )

    # DAC enable & full-scale current (match ad9081)
    @property
    def tx_dac_en(self):
        return self._get_iio_attr_vec(self._tx_coarse_duc_channel_names, "en", True)

    @tx_dac_en.setter
    def tx_dac_en(self, value):
        self._set_iio_attr_int_vec(
            self._tx_coarse_duc_channel_names, "en", True, value
        )

    def set_tx_dac_full_scale_current(self, value):
        # microamps, via debugfs like ad9081
        self._set_iio_debug_attr_str(
            "dac-full-scale-current-ua", str(value), self._rxadc
        )

    tx_dac_full_scale_current = property(None, set_tx_dac_full_scale_current)

    # Sample-rate / clocks (identical semantics to ad9081)
    @property
    def rx_sample_rate(self):
        """Sample rate after decimation"""
        return self._get_iio_attr_single(
            self._rx_coarse_ddc_channel_names, "sampling_frequency", False
        )

    @property
    def adc_frequency(self):
        """ADC frequency in Hz"""
        return self._get_iio_attr_single(
            self._rx_coarse_ddc_channel_names, "adc_frequency", False
        )

    @property
    def tx_sample_rate(self):
        """Sample rate before interpolation"""
        return self._get_iio_attr_single(
            self._tx_coarse_duc_channel_names, "sampling_frequency", True
        )

    @property
    def dac_frequency(self):
        """DAC frequency in Hz"""
        return self._get_iio_attr_single(
            self._tx_coarse_duc_channel_names, "dac_frequency", True
        )

    # JESD / status (unchanged)
    @property
    def jesd204_fsm_ctrl(self):
        return self._get_iio_dev_attr("jesd204_fsm_ctrl", self._rxadc)

    @jesd204_fsm_ctrl.setter
    def jesd204_fsm_ctrl(self, value):
        self._set_iio_dev_attr("jesd204_fsm_ctrl", value, self._rxadc)

    @property
    def jesd204_fsm_resume(self):
        return self._get_iio_dev_attr("jesd204_fsm_resume", self._rxadc)

    @jesd204_fsm_resume.setter
    def jesd204_fsm_resume(self, value):
        self._set_iio_dev_attr_str("jesd204_fsm_resume", value, self._rxadc)

    @property
    def jesd204_fsm_state(self):
        return self._get_iio_dev_attr_str("jesd204_fsm_state", self._rxadc)

    @property
    def jesd204_fsm_paused(self):
        return self._get_iio_dev_attr("jesd204_fsm_paused", self._rxadc)

    @property
    def jesd204_fsm_error(self):
        return self._get_iio_dev_attr("jesd204_fsm_error", self._rxadc)

    @property
    def jesd204_device_status(self):
        return self._get_iio_debug_attr_str("status", self._rxadc)

    @property
    def jesd204_device_status_check(self):
        stat = self._get_iio_debug_attr_str("status", self._rxadc)
        for s in stat.splitlines(0):
            if "JRX" in s:
                if "204C" in s:
                    if "Link is good" not in s:
                        return True
                elif "204B" in s:
                    if "0x0 lanes in DATA" in s:
                        return True
            elif "JTX" in s:
                if any(substr in s for substr in
                       [" asserted", "unlocked", "lost", "invalid"]):
                    return True
        return False

    @property
    def chip_version(self):
        return self._get_iio_debug_attr_str("chip_version", self._rxadc)

    @property
    def api_version(self):
        return self._get_iio_debug_attr_str("api_version", self._rxadc)

    # Optional: powerdown (same as ad9081) if your driver exposes it
    @property
    def powerdown(self):
        return self._get_iio_dev_attr_single("powerdown")

    @powerdown.setter
    def powerdown(self, value):
        self._set_iio_dev_attr_single("powerdown", value)
