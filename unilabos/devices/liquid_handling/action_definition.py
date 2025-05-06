from __future__ import annotations

from typing import List, Sequence, Optional, Literal, Union, Iterator

import asyncio
import time

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.resources import (
    Resource,
    TipRack,
    Container,
    Coordinate,
    Well
)

class DPLiquidHandler(LiquidHandler):
    """Extended LiquidHandler with additional operations."""

    # ---------------------------------------------------------------
    # REMOVE LIQUID --------------------------------------------------
    # ---------------------------------------------------------------

    async def remove_liquid(
        self,
        vols: List[float],
        sources: Sequence[Container],
        waste_liquid: Optional[Container] = None,
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        delays: Optional[List[int]] = None,
        is_96_well: Optional[bool] = False,
        top: Optional[List(float)] = None,
    ):
        """A complete *remove* (aspirate → waste) operation."""
        trash = self.deck.get_trash_area()
        try:
            if is_96_well:
                pass # This mode is not verified
            else:
                if len(vols) != len(sources):
                    raise ValueError("Length of `vols` must match `sources`.")

                for src, vol in zip(sources, vols):
                    self.move_to(src, dis_to_top=top[0] if top else 0)
                    tip = next(self.current_tip)
                    await self.pick_up_tips(tip)
                    await self.aspirate(
                        resources=[src],
                        vols=[vol],
                        use_channels=use_channels, # only aspirate96 used, default to None
                        flow_rates=[flow_rates[0]] if flow_rates else None,
                        offsets=[offsets[0]] if offsets else None,
                        liquid_height=[liquid_height[0]] if liquid_height else None,
                        blow_out_air_volume=blow_out_air_volume[0] if blow_out_air_volume else None,
                        spread=spread,
                    )
                    await self.custom_delay(seconds=delays[0] if delays else 0)
                    await self.dispense(
                        resources=waste_liquid,
                         vols=[vol],
                         use_channels=use_channels,
                         flow_rates=[flow_rates[1]] if flow_rates else None,
                         offsets=[offsets[1]] if offsets else None,
                         liquid_height=[liquid_height[1]] if liquid_height else None,
                         blow_out_air_volume=blow_out_air_volume[1] if blow_out_air_volume else None,
                         spread=spread,
                     )
                    await self.discard_tips() # For now, each of tips is discarded after use

        except Exception as e:
            raise RuntimeError(f"Liquid removal failed: {e}") from e

    # ---------------------------------------------------------------
    # ADD LIQUID -----------------------------------------------------
    # ---------------------------------------------------------------

    async def add_liquid(
        self,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        reagent_sources: Sequence[Container],
        targets: Sequence[Container],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        is_96_well: bool = False,
        delays: Optional[List[int]] = None,
        mix_time: Optional[int] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None
    ):
        """A complete *add* (aspirate reagent → dispense into targets) operation."""

        try:
            if is_96_well:
                pass # This mode is not verified.
            else:
                if len(asp_vols) != len(targets):
                    raise ValueError("Length of `vols` must match `targets`.")
                tip = next(self.current_tip)
                await self.pick_up_tips(tip)

                for _ in range(len(targets)):
                    await self.aspirate(
                        resources=reagent_sources,
                        vols=[asp_vols[_]],
                        use_channels=use_channels,
                        flow_rates=[flow_rates[0]] if flow_rates else None,
                        offsets=[offsets[0]] if offsets else None,
                        liquid_height=[liquid_height[0]] if liquid_height else None,
                        blow_out_air_volume=[blow_out_air_volume[0]] if blow_out_air_volume else None,
                        spread=spread
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[0])
                    await self.dispense(
                        resources=[targets[_]],
                        vols=[dis_vols[_]],
                        use_channels=use_channels,
                        flow_rates=[flow_rates[1]] if flow_rates else None,
                        offsets=[offsets[1]] if offsets else None,
                        blow_out_air_volume=[blow_out_air_volume[1]] if blow_out_air_volume else None,
                        liquid_height=[liquid_height[1]] if liquid_height else None,
                        spread=spread,
                    )
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])
                    await self.mix(
                        targets=targets[_],
                        mix_time=mix_time,
                        mix_vol=mix_vol,
                        offsets=offsets if offsets else None,
                        height_to_bottom=mix_liquid_height if mix_liquid_height else None,
                        mix_rate=mix_rate if mix_rate else None)
                    if delays is not None:
                        await self.custom_delay(seconds=delays[1])
                    await self.touch_tip(targets[_])
                await self.discard_tips()

        except Exception as e:
            raise RuntimeError(f"Liquid addition failed: {e}") from e

    # ---------------------------------------------------------------
    # TRANSFER LIQUID ------------------------------------------------
    # ---------------------------------------------------------------
    async def transfer_liquid(
        self,
        asp_vols: Union[List[float], float],
        dis_vols: Union[List[float], float],
        sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
        *,
        use_channels: Optional[List[int]] = None,
        asp_flow_rates: Optional[List[Optional[float]]] = None,
        dis_flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        touch_tip: bool = False,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        is_96_well: bool = False,
        mix_stage: Optional[Literal["none", "before", "after", "both"]] = "none",
        mix_times: Optional[List(int)] = None,
        mix_vol: Optional[int] = None,
        mix_rate: Optional[int] = None,
        mix_liquid_height: Optional[float] = None,
        delays: Optional[List[int]] = None,
    ):
        """Transfer liquid from each *source* well/plate to the corresponding *target*.

        Parameters
        ----------
        asp_vols, dis_vols
            Single volume (µL) or list matching the number of transfers.
        sources, targets
            Same‑length sequences of containers (wells or plates). In 96‑well mode
            each must contain exactly one plate.
        tip_racks
            One or more TipRacks providing fresh tips.
        is_96_well
            Set *True* to use the 96‑channel head.
        """

        try:
            # ------------------------------------------------------------------
            # 96‑channel head mode
            # ------------------------------------------------------------------
            if is_96_well:
                pass # This mode is not verified
            else:
                if not (len(asp_vols) == len(sources) and len(dis_vols) == len(targets)):
                    raise ValueError("`sources`, `targets`, and `vols` must have the same length.")

                tip_iter = self.iter_tips(tip_racks)
                for src, tgt, asp_vol, asp_flow_rate, dis_vol, dis_flow_rate in (
                        zip(sources, targets, asp_vols, asp_flow_rates, dis_vols, dis_flow_rates)):
                    tip = next(tip_iter)
                    await self.pick_up_tips(tip)
                    # Aspirate from source
                    await self.aspirate(
                        resources=[src],
                        vols=[asp_vol],
                        use_channels=use_channels,
                        flow_rates=[asp_flow_rate],
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    self.custom_delay(seconds=delays[0] if delays else 0)
                    # Dispense into target
                    await self.dispense(
                        resources=[tgt],
                        vols=[dis_vol],
                        use_channels=use_channels,
                        flow_rates=[dis_flow_rate],
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    await self.mix(
                        targets=[tgt],
                        mix_time=mix_times[0] if mix_times else None,
                        mix_vol=mix_vol[0] if mix_vol else None,
                        mix_rate=mix_rate[0] if mix_rate else None,
                    )
                    if touch_tip:
                        await self.touch_tip(tgt)
                    await self.discard_tips()

        except Exception as exc:
            raise RuntimeError(f"Liquid transfer failed: {exc}") from exc

# ---------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------

    async def custom_delay(self, seconds=0, msg=None):
        """
        seconds: seconds to wait
        msg: information to be printed
        """
        if seconds != None and seconds > 0:
            if msg:
                print(f"Waiting time: {msg}")
                print(f"Current time: {time.strftime('%H:%M:%S')}")
                print(f"Time to finish: {time.strftime('%H:%M:%S', time.localtime(time.time() + seconds))}")
            await asyncio.sleep(seconds)
            if msg:
                print(f"Done: {msg}")
                print(f"Current time: {time.strftime('%H:%M:%S')}")

    async def touch_tip(self,
                        targets: Sequence[Container],
                        ):
        """Touch the tip to the side of the well."""
        await self.aspirate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=[Coordinate(x=-targets.get_size_x()/2,y=0,z=0)],
            liquid_height=None,
            blow_out_air_volume=None
        )
        #await self.custom_delay(seconds=1) # In the simulation, we do not need to wait
        await self.aspirate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=[Coordinate(x=targets.get_size_x()/2,y=0,z=0)],
            liquid_height=None,
            blow_out_air_volume=None
        )

    async def mix(
        self,
        targets: Sequence[Container],
        mix_time: int = None,
        mix_vol: Optional[int] = None,
        height_to_bottom: Optional[float] = None,
        offsets: Optional[Coordinate] = None,
        mix_rate: Optional[float] = None,
    ):
        if mix_time is None: # No mixing required
            return
        """Mix the liquid in the target wells."""
        for _ in range(mix_time):
            await self.aspirate(
                resources=[targets],
                vols=[mix_vol],
                flow_rates=[mix_rate] if mix_rate else None,
                offsets=[offsets] if offsets else None,
                liquid_height=[height_to_bottom] if height_to_bottom else None,
            )
            await self.custom_delay(seconds=1)
            await self.dispense(
                resources=[targets],
                vols=[mix_vol],
                flow_rates=[mix_rate] if mix_rate else None,
                offsets=[offsets] if offsets else None,
                liquid_height=[height_to_bottom] if height_to_bottom else None,
            )

    def iter_tips(self, tip_racks: Sequence[TipRack]) -> Iterator[Resource]:
        """Yield tips from a list of TipRacks one-by-one until depleted."""
        for rack in tip_racks:
            for tip in rack:
                yield tip
        raise RuntimeError("Out of tips!")

    def set_tiprack(self, tip_racks: Sequence[TipRack]):
        """Set the tip racks for the liquid handler."""
        self.tip_racks = tip_racks
        tip_iter = self.iter_tips(tip_racks)
        self.current_tip = tip_iter

    async def move_to(self, well: Well, dis_to_top: float = 0 , channel: int = 0):
        """
        Move a single channel to a specific well with a given z-height.

        Parameters
        ----------
        well : Well
            The target well.
        dis_to_top : float
            Height in mm to move to relative to the well top.
        channel : int
            Pipetting channel to move (default: 0).
        """
        await self.prepare_for_manual_channel_operation(channel=channel)
        abs_loc = well.get_absolute_location()
        well_height = well.get_absolute_size_z()
        await self.move_channel_x(channel, abs_loc.x)
        await self.move_channel_y(channel, abs_loc.y)
        await self.move_channel_z(channel, abs_loc.z + well_height + dis_to_top)

