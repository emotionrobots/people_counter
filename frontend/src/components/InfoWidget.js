import React, { useState, useContext, useEffect } from 'react';
import PropTypes from 'prop-types';

import Card from './Card';
import Loading from './InfoWidgetTypes/Loading';
import Report from './InfoWidgetTypes/Report';
import Single from './InfoWidgetTypes/Single';
import { getInfoWidget } from '../data/user_data';
import Chart from './InfoWidgetTypes/Chart';
import Settings from './InfoWidgetTypes/Settings';
import ListAndInfo from './InfoWidgetTypes/ListAndInfo';
import { StateContext } from './Contexts/StateContext';

export const InfoWidgetTypes = {
    SINGLE: "infocard.single",
    REPORT: "infocard.report",
    CHART: "infocard.chart",
    LOADING: "infocard.waiting",
    SETTINGS: "infocard.settings",
    LISTPLUSINFO: "infocard.listinfo"
}

    function renderBody(state) {
        switch (state.cardType) {
            case InfoWidgetTypes.LOADING:
                return <Loading />;
            case InfoWidgetTypes.REPORT:
                return <Report data={state.attributes.data}/>;
            case InfoWidgetTypes.SINGLE:
                return <Single data={state.attributes} />;
            case InfoWidgetTypes.CHART:
                return <Chart data={state.attributes.data} />;
            case InfoWidgetTypes.SETTINGS:
                return <Settings data={state.attributes.data}/>;
            case InfoWidgetTypes.LISTPLUSINFO:
                return <ListAndInfo data={state.attributes.data} horz={state.attributes.horz}></ListAndInfo>;
            default:
                break;
        }
    }

export function InfoWidget(props) {
    const [global_state,] = useContext(StateContext);
    const [state, setState] = useState({
        cardType: InfoWidgetTypes.LOADING,
    })

    const setInfoWidget = () => {
        getInfoWidget(props.data, (ret) => {
            if(ret === "Error") {
                setState({
                    cardType: InfoWidgetTypes.SINGLE,
                    attributes: {
                        data: "Error"
                    }
                })
            } else {
                setState(ret)
            }
        }, global_state.currentSelectedCamGroup)
    }

    useEffect(setInfoWidget, [
        props.data,
        global_state.currentSelectedCamGroup
    ])

    return (
        <Card bgColor={props.bgColor}>
            {renderBody(state)}
        </Card>
    );
}

InfoWidget.propTypes = {
    data: PropTypes.string,
    bgColor: PropTypes.string
};

export default InfoWidget;