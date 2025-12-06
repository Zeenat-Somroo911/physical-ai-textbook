import React from 'react';
import { useAuth } from '../../context/AuthContext';
import { useLocation } from '@docusaurus/router';
import styles from '../../css/custom.css';

export default function AuthButton() {
    const { user, logout } = useAuth();
    const location = useLocation();

    React.useEffect(() => {
        console.log('[AuthButton] Rendered. User state:', user);
    }, [user]);

    const handleLogout = () => {
        console.log('[AuthButton] Logout clicked');
        logout();
        window.location.reload();
    };

    const openAuthModal = (mode) => {
        window.location.hash = mode === 'signup' ? '#signup' : '#signin';
    };

    if (user) {
        return (
            <div className="navbar-auth-container" style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                <span style={{ fontSize: '0.9rem', fontWeight: 500, display: 'none', lg: 'block' }}>
                    Hi, {user.full_name?.split(' ')[0] || 'User'}
                </span>
                <button
                    onClick={handleLogout}
                    className="navbar-signup-opts"
                    style={{
                        background: 'transparent !important',
                        border: '1px solid var(--ifm-color-danger) !important',
                        backgroundImage: 'none !important',
                        color: 'var(--ifm-color-danger) !important'
                    }}
                >
                    Logout
                </button>
            </div>
        );
    }

    return (
        <button
            onClick={() => openAuthModal('signin')}
            className="navbar-signup-opts"
        >
            Sign In
        </button>
    );
}
