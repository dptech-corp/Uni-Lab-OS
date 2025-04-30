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
)

class MyLiquidHandler(LiquidHandler):
    """Extended LiquidHandler with additional operations."""
    
    # ---------------------------------------------------------------
    # REMOVE LIQUID --------------------------------------------------
    # ---------------------------------------------------------------

    async def remove_liquid(
        self,
        vols: List[float],
        sources: Sequence[Container],
        tip_racks: Sequence[TipRack],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Optional[Literal["wide", "tight", "custom"]] = "wide",
        is_96_well: Optional[bool] = False,
    ):
        """A complete *remove* (aspirate → waste) operation."""
        trash = self.deck.get_trash_area()
        try:
            if is_96_well:
                if not isinstance(vols, (int, float)):
                    raise ValueError("For 96‑well operations `vols` must be a scalar.")
                first_rack = next(iter(tip_racks))
                await self.pick_up_tips96(first_rack)
                await self.aspirate96(
                    resource=sources,
                    volume=vols,
                    offset=Coordinate.zero(),
                    flow_rate=flow_rates[0] if flow_rates else None,
                    blow_out_air_volume=blow_out_air_volume[0] if blow_out_air_volume else None,
                    use_channels=use_channels,
                )
                await self.dispense(
                    resources=[trash],
                    vols=[vols] * 8,
                    use_channels=use_channels,
                    flow_rates=flow_rates,
                    offsets=None,
                    liquid_height=liquid_height,
                    blow_out_air_volume=blow_out_air_volume,
                    spread=spread,
                )
                await self.discard_tips96()
            else:
                if len(vols) != len(sources):
                    raise ValueError("Length of `vols` must match `sources`.")
                tip_iter = self.iter_tips(tip_racks)
                for src, vol in zip(sources, vols):
                    tip = next(tip_iter)
                    await self.pick_up_tips(tip)
                    await self.aspirate(
                        resources=[src],
                        vols=[vol],
                        use_channels=use_channels, # only aspirate96 used, default to None
                        flow_rates=flow_rates[0] if flow_rates else None,
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    await self.dispense(
                        resources=[trash],
                        vols=[vol],
                        use_channels=use_channels,
                        flow_rates=None,
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
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
        vols: Union[List[float], float],
        reagent_sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        is_96_well: bool = False,
        delays: Optional[List[int]] = None
        mix_times = Optional[List[int]] = None,
        mix_reps = Optional[List[int]] = None,
        mix_vol = Optional[List[int]] = None,
    ):
        """A complete *add* (aspirate reagent → dispense into targets) operation."""

        try:
            if is_96_well:
                if not isinstance(vols, (int, float)):
                    raise ValueError("For 96‑well operations `vols` must be a scalar.")
                first_rack = next(iter(tip_racks))
                await self.pick_up_tips96(first_rack)
                await self.aspirate(
                    resources=reagent_sources,
                    vols=[vols],
                    use_channels=use_channels,
                    flow_rates=flow_rates,
                    offsets=offsets,
                    liquid_height=liquid_height,
                    blow_out_air_volume=blow_out_air_volume,
                    spread=spread,
                )
                await self.dispense96(
                    resource=targets,
                    volume=vols,
                    offset=Coordinate.zero(),
                    flow_rate=flow_rates[0] if flow_rates else None,
                    blow_out_air_volume=blow_out_air_volume[0] if blow_out_air_volume else None,
                    use_channels=use_channels,
                )
                await self.discard_tips96()
            else:
                if len(vols) != len(targets):
                    raise ValueError("Length of `vols` must match `targets`.")
                tip_iter = self.iter_tips(tip_racks)
                for tgt, vol in zip(targets, vols):
                    tip = next(tip_iter)
                    await self.pick_up_tips(tip)
                    await self.aspirate(
                        resources=reagent_sources,
                        vols=[vol],
                        use_channels=use_channels,
                        flow_rates=flow_rates,
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    await self.custom_delay(seconds=delays[0] if delays else 0)
                    await self.dispense(
                        resources=[tgt],
                        vols=[vol],
                        use_channels=use_channels,
                        flow_rates=flow_rates,
                        offsets=offsets,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    await self.touch_tip(tgt)
                    await self.discard_tips()
            raise RuntimeError(f"Liquid addition failed: {e}") from e

    # ---------------------------------------------------------------
    # TRANSFER LIQUID ------------------------------------------------
    # ---------------------------------------------------------------
    async def transfer_liquid(
        self,
        vols: Union[float, List[float]],
        sources: Sequence[Container],
        targets: Sequence[Container],
        tip_racks: Sequence[TipRack],
        *,
        use_channels: Optional[List[int]] = None,
        flow_rates: Optional[List[Optional[float]]] = None,
        offsets: Optional[List[Coordinate]] = None,
        liquid_height: Optional[List[Optional[float]]] = None,
        blow_out_air_volume: Optional[List[Optional[float]]] = None,
        spread: Literal["wide", "tight", "custom"] = "wide",
        is_96_well: bool = False,
        mix_times: int = None,
        mix_vol: Optional[int] = None,
        delays: Optional[List[int]] = None,
    ):
        """Transfer liquid from each *source* well/plate to the corresponding *target*.

        Parameters
        ----------
        vols
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
                # Validate inputs ------------------------------------------------

                if len(sources) != 1 or len(targets) != 1:
                    raise ValueError("Provide exactly one source plate and one target plate in 96‑well mode.")

                # 1) Pick up 96 tips
                first_rack = next(iter(tip_racks))
                await self.pick_up_tips96(first_rack)

                # 2) Aspirate from source plate
                await self.aspirate96(
                    resource=sources,
                    volume=vols,
                    offset=Coordinate.zero(),
                    flow_rate=flow_rates[0] if flow_rates else None,
                    blow_out_air_volume=blow_out_air_volume[0] if blow_out_air_volume else None,
                    use_channels=use_channels,
                )

                # 3) Dispense into target plate
                await self.dispense96(
                    resource=targets,
                    volume=vols,
                    offset=Coordinate.zero(),
                    flow_rate=flow_rates[0] if flow_rates else None,
                    blow_out_air_volume=blow_out_air_volume[0] if blow_out_air_volume else None,
                    use_channels=use_channels,
                )

                # 4) Drop tips
                await self.discard_tips96()
                return  # success
            
            else:
                if not (len(vols) == len(sources) == len(targets)):
                    raise ValueError("`sources`, `targets`, and `vols` must have the same length.")

                tip_iter = self.iter_tips(tip_racks)
                for src, tgt, vol in zip(sources, targets, vols):
                    tip = next(tip_iter)
                    await self.pick_up_tips(tip)
                    # Aspirate from source
                    await self.aspirate(
                        resources=[src],
                        vols=[vol],
                        use_channels=use_channels,
                        flow_rates=flow_rates,
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    self.custom_delay(seconds=delays[0] if delays else 0)
                    # Dispense into target
                    await self.dispense(
                        resources=[tgt],
                        vols=[vol],
                        use_channels=use_channels,
                        flow_rates=flow_rates,
                        offsets=offsets,
                        liquid_height=liquid_height,
                        blow_out_air_volume=blow_out_air_volume,
                        spread=spread,
                    )
                    await self.mix(
                        targets=[tgt],
                        mix_times=mix_times,
                        mix_vol=mix_vol)
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

        if seconds > 0:
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
        await self.asperate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=Coordinate(x=targets.size_x/2),
            liquid_height=None,
            blow_out_air_volume=None,
            spread="wide"
        )
        await self.custom_delay(seconds=1)
        await self.asperate(
            resources=[targets],
            vols=[0],
            use_channels=None,
            flow_rates=None,
            offsets=Coordinate(x=-targets.size_x/2),
            liquid_height=None,
            blow_out_air_volume=None,
            spread="wide"
        )
    async def mix(
        self,
        targets: Sequence[Container],
        mix_times: int = None,
        mix_vol: Optional[int] = None,
    ):
        """Mix the liquid in the target wells."""
        for _ in mix_times:
            for target in targets:
                await self.aspirate(
                    resources=[target],
                    vols=[mix_vol],
                    use_channels=None,
                    flow_rates=None,
                    offsets=None,
                    liquid_height=None,
                    blow_out_air_volume=None,
                    spread="wide"
                )
                await self.custom_delay(seconds=1)
                await self.aspirate(
                    resources=[target],
                    vols=[mix_vol],
                    use_channels=None,
                    flow_rates=None,
                    offsets=None,
                    liquid_height=None,
                    blow_out_air_volume=None,
                    spread="wide"
                )

    def iter_tips(self, tip_racks: Sequence[TipRack]) -> Iterator[Resource]:
        """Yield tips from a list of TipRacks one-by-one until depleted."""
        for rack in tip_racks:
            for tip in rack:
                yield tip
        raise RuntimeError("Out of tips!")
