export const GlobalAppReducer = (state, action) => {
    switch (action.type) {
        case "start_loading":
            return {
                ...state,
                isLoading: true,
                loadingMessage: action.loadingMessage
            }
        case "stop_loading":
            return {
                ...state,
                isLoading: false,
            }
        case "switch_cam_group":
            return {
                ...state,
                currentSelectedCamGroup: action.camGroup
            }
        default:
            return state
    }
}

export const initialState = {
    isLoading: false,
    loadingMessage: null,
    currentSelectedCamGroup: [0, 0],
}