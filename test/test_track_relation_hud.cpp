#include "trackcontrol.h"

#include <iostream>
#include <string>
#include <vector>

static std::vector<int> makeMid(int h = 240)
{
    return std::vector<int>(h, -1);
}

static bool expect(bool cond, const std::string& msg)
{
    if (!cond) {
        std::cerr << msg << "\n";
        return false;
    }
    return true;
}

int main()
{
    const std::string inside =
        tcFormatTrackRelationHud(230, 170, 160, -80, 80);
    const std::string outside =
        tcFormatTrackRelationHud(230, 260, 160, -80, 80);
    const std::string invalid =
        tcFormatTrackRelationHud(230, -1, 160, -80, 80);
    const std::string swapped =
        tcFormatTrackRelationHud(230, 90, 160, 80, -80);

    if (inside != "TRACK_REL y=230 err=10 IN") {
        std::cerr << "bad inside hud: " << inside << "\n";
        return 1;
    }
    if (outside != "TRACK_REL y=230 err=100 OUT") {
        std::cerr << "bad outside hud: " << outside << "\n";
        return 2;
    }
    if (invalid != "TRACK_REL y=230 err=-- BAD") {
        std::cerr << "bad invalid hud: " << invalid << "\n";
        return 3;
    }
    if (swapped != "TRACK_REL y=230 err=-70 IN") {
        std::cerr << "bad swapped threshold hud: " << swapped << "\n";
        return 4;
    }

    TrackControlParams tc;
    tc.carTrackRelationY = 230;
    tc.carTrackInsideErrorMin = -80;
    tc.carTrackInsideErrorMax = 80;
    tc.carTrackOutsideEnterConfirmFrames = 1;
    tc.carTrackInsideEnterConfirmFrames = 1;

    {
        auto mid = makeMid();
        mid[225] = 160;
        mid[230] = 160;
        mid[235] = 260;
        TcTrackRelationState state;
        state.initialized = true;
        state.inside = true;
        const TcTrackRelationResult rel =
            tcEvaluateTrackRelation(mid, 160, tc, &state);
        if (!expect(rel.valid && rel.inside, "single bottom row spike should hold IN"))
            return 5;
        const std::string hud = tcFormatTrackRelationHud(rel);
        if (!expect(hud == "REL y230 e50 IN sc3/3 cfgO1/I1",
                    "stable HUD should label score and config"))
            return 6;
    }

    {
        auto mid = makeMid();
        mid[225] = 160;
        mid[230] = 250;
        mid[235] = 260;
        TcTrackRelationState state;
        state.initialized = true;
        state.inside = true;
        const TcTrackRelationResult rel =
            tcEvaluateTrackRelation(mid, 160, tc, &state);
        if (!expect(rel.valid && !rel.inside, "two near rows outside should switch OUT"))
            return 7;
    }

    {
        auto mid = makeMid();
        mid[225] = 160;
        mid[230] = 250;
        mid[235] = 260;
        tc.carTrackOutsideEnterConfirmFrames = 3;
        const TcTrackRelationResult rel =
            tcEvaluateTrackRelation(mid, 160, tc, nullptr);
        if (!expect(rel.valid && !rel.inside && rel.pending_count == 0,
                    "stateless relation evaluation should switch immediately"))
            return 8;
        tc.carTrackOutsideEnterConfirmFrames = 1;
    }

    {
        auto mid = makeMid();
        mid[225] = 160;
        mid[230] = 250;
        mid[235] = 260;
        TcTrackRelationState state;
        state.initialized = true;
        state.inside = false;
        const TcTrackRelationResult rel =
            tcEvaluateTrackRelation(mid, 160, tc, &state);
        if (!expect(rel.valid && !rel.inside, "weak inside vote should hold OUT"))
            return 9;

        mid[230] = 160;
        mid[235] = 160;
        const TcTrackRelationResult rel2 =
            tcEvaluateTrackRelation(mid, 160, tc, &state);
        if (!expect(rel2.valid && rel2.inside, "strong inside vote should switch IN"))
            return 10;
    }

    {
        auto mid = makeMid();
        TcTrackRelationState state;
        state.initialized = true;
        state.inside = true;
        const TcTrackRelationResult rel =
            tcEvaluateTrackRelation(mid, 160, tc, &state);
        if (!expect(!rel.valid && rel.inside, "invalid sample should hold previous state"))
            return 11;
    }

    {
        tc.carTrackOutsideEnterConfirmFrames = 3;
        tc.carTrackInsideEnterConfirmFrames = 2;

        auto outside_mid = makeMid();
        outside_mid[225] = 160;
        outside_mid[230] = 250;
        outside_mid[235] = 260;
        TcTrackRelationState state;
        state.initialized = true;
        state.inside = true;

        TcTrackRelationResult rel =
            tcEvaluateTrackRelation(outside_mid, 160, tc, &state);
        if (!expect(rel.valid && rel.inside && rel.pending_count == 1,
                    "first OUT candidate should still hold IN"))
            return 12;
        const std::string pending_hud = tcFormatTrackRelationHud(rel);
        if (!expect(pending_hud ==
                        "REL y230 e80 IN sc1/5 toOUT1/3",
                    "pending HUD should label score, transition, and config"))
            return 13;

        auto invalid_during_pending = makeMid();
        rel = tcEvaluateTrackRelation(invalid_during_pending, 160, tc, &state);
        if (!expect(!rel.valid && rel.inside && rel.pending_count == 0,
                    "invalid sample should reset pending OUT confirmation"))
            return 14;

        rel = tcEvaluateTrackRelation(outside_mid, 160, tc, &state);
        if (!expect(rel.valid && rel.inside && rel.pending_count == 1,
                    "OUT confirmation should restart after invalid sample"))
            return 15;

        rel = tcEvaluateTrackRelation(outside_mid, 160, tc, &state);
        if (!expect(rel.valid && rel.inside && rel.pending_count == 2,
                    "second OUT candidate should still hold IN"))
            return 16;

        rel = tcEvaluateTrackRelation(outside_mid, 160, tc, &state);
        if (!expect(rel.valid && !rel.inside && rel.pending_count == 0,
                    "third OUT candidate should switch OUT"))
            return 17;

        auto invalid_mid = makeMid();
        rel = tcEvaluateTrackRelation(invalid_mid, 160, tc, &state);
        if (!expect(!rel.valid && !rel.inside && rel.pending_count == 0,
                    "invalid sample should not advance IN confirmation"))
            return 18;

        auto inside_mid = makeMid();
        inside_mid[225] = 160;
        inside_mid[230] = 160;
        inside_mid[235] = 160;
        rel = tcEvaluateTrackRelation(inside_mid, 160, tc, &state);
        if (!expect(rel.valid && !rel.inside && rel.pending_count == 1,
                    "first IN candidate should still hold OUT"))
            return 19;

        rel = tcEvaluateTrackRelation(inside_mid, 160, tc, &state);
        if (!expect(rel.valid && rel.inside && rel.pending_count == 0,
                    "second IN candidate should switch IN"))
            return 20;
    }

    std::cout << "track relation HUD formatting passed\n";
    return 0;
}
