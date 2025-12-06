import React, { useState, useEffect } from 'react';
import { FaEnvelope, FaLock, FaUser, FaTimes, FaEye, FaEyeSlash } from 'react-icons/fa';
import styles from './AuthModal.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../context/AuthContext';

const AuthModal = ({ isOpen, onClose, initialMode = 'signup' }) => {
    const [isLogin, setIsLogin] = useState(initialMode === 'signin');
    const [formData, setFormData] = useState({
        fullName: '',
        email: '',
        password: '',
        background: 'Student'
    });
    const { siteConfig } = useDocusaurusContext();
    const { login } = useAuth();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');
    const [success, setSuccess] = useState('');
    const [showPassword, setShowPassword] = useState(false);

    // Update mode when initialMode changes
    useEffect(() => {
        setIsLogin(initialMode === 'signin');
    }, [initialMode]);

    // Reset form when modal opens
    useEffect(() => {
        if (isOpen) {
            resetForm();
        }
    }, [isOpen]);

    if (!isOpen) return null;

    const handleChange = (e) => {
        setFormData({ ...formData, [e.target.name]: e.target.value });
        setError(''); // Clear error on input change
    };

    const resetForm = () => {
        setFormData({
            fullName: '',
            email: '',
            password: '',
            background: 'Student'
        });
        setError('');
        setSuccess('');
        setShowPassword(false);
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        setError('');
        setSuccess('');

        const API_URL = siteConfig.customFields?.chatbotUrl || 'http://localhost:8000';
        const endpoint = isLogin ? '/auth/login' : '/auth/register';

        try {
            const payload = isLogin
                ? { email: formData.email, password: formData.password }
                : {
                    email: formData.email,
                    password: formData.password,
                    full_name: formData.fullName,
                    preferences: { background: formData.background }
                };

            const response = await fetch(`${API_URL}${endpoint}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(payload),
            });

            const data = await response.json();

            if (!response.ok) {
                if (data.detail && data.detail.includes('Email already registered')) {
                    throw new Error('This email is already registered. Try signing in instead or use a different email.');
                } else if (data.detail && data.detail.includes('Invalid email or password')) {
                    throw new Error('Invalid email or password. Please check your credentials.');
                } else {
                    throw new Error(data.detail || 'Something went wrong. Please try again.');
                }
            }

            if (isLogin) {
                setSuccess(`Welcome back, ${data.user.full_name || 'User'}! Redirecting...`);
                setTimeout(() => {
                    login(data.user, data.access_token);
                    onClose();
                }, 1500);
            } else {
                setSuccess('Account created successfully! Please Sign In.');
                setTimeout(() => {
                    setIsLogin(true);
                    setSuccess('');
                    setFormData(prev => ({ ...prev, password: '' }));
                }, 1500);
            }

        } catch (err) {
            console.error('Auth error:', err);
            setError(err.message || 'Something went wrong. Please try again.');
        } finally {
            setLoading(false);
        }
    };

    const toggleMode = () => {
        setIsLogin(!isLogin);
        resetForm();
    };

    const togglePasswordVisibility = () => {
        setShowPassword(!showPassword);
    };

    return (
        <div className={styles.overlay} onClick={onClose}>
            <div className={styles.modalContainer}>
                <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
                    <button className={styles.closeButton} onClick={onClose}>
                        <FaTimes />
                    </button>

                    <div className={styles.header}>
                        <h2 className={styles.title}>
                            {isLogin ? 'Welcome Back' : 'Create Account'}
                        </h2>
                        <p className={styles.subtitle}>
                            {isLogin
                                ? 'Enter your details to access your personalized learning path.'
                                : 'Join us to track your progress and master Physical AI.'}
                        </p>
                    </div>

                    <form className={styles.form} onSubmit={handleSubmit}>
                        {!isLogin && (
                            <>
                                <div className={styles.inputGroup}>
                                    <FaUser className={styles.icon} />
                                    <input
                                        type="text"
                                        name="fullName"
                                        placeholder="Full Name"
                                        className={styles.input}
                                        value={formData.fullName}
                                        onChange={handleChange}
                                        required
                                    />
                                </div>
                                <div className={styles.inputGroup}>
                                    <FaUser className={styles.icon} />
                                    <select
                                        name="background"
                                        value={formData.background}
                                        onChange={handleChange}
                                        className={styles.input}
                                        style={{ backgroundColor: 'var(--ifm-background-color)', color: 'var(--ifm-font-color-base)' }}
                                    >
                                        <option value="Student">Student (Learning basics)</option>
                                        <option value="Engineer">Engineer (Professional)</option>
                                        <option value="Researcher">Researcher (Academic)</option>
                                        <option value="Hobbyist">Hobbyist (Maker)</option>
                                    </select>
                                </div>
                            </>
                        )}

                        <div className={styles.inputGroup}>
                            <FaEnvelope className={styles.icon} />
                            <input
                                type="email"
                                name="email"
                                placeholder="Email Address"
                                className={styles.input}
                                value={formData.email}
                                onChange={handleChange}
                                required
                            />
                        </div>

                        <div className={styles.inputGroup} style={{ position: 'relative' }}>
                            <FaLock className={styles.icon} />
                            <input
                                type={showPassword ? 'text' : 'password'}
                                name="password"
                                placeholder="Password (min 8 characters)"
                                className={styles.input}
                                value={formData.password}
                                onChange={handleChange}
                                required
                                minLength={8}
                                style={{ paddingRight: '3rem' }}
                            />
                            <button
                                type="button"
                                onClick={togglePasswordVisibility}
                                style={{
                                    position: 'absolute',
                                    right: '1rem',
                                    top: '50%',
                                    transform: 'translateY(-50%)',
                                    background: 'none',
                                    border: 'none',
                                    cursor: 'pointer',
                                    color: 'var(--ifm-color-emphasis-600)',
                                    fontSize: '1.1rem',
                                    padding: '0.5rem',
                                    display: 'flex',
                                    alignItems: 'center',
                                    justifyContent: 'center',
                                    transition: 'color 0.2s ease',
                                    zIndex: 10
                                }}
                                onMouseEnter={(e) => e.currentTarget.style.color = 'var(--ifm-color-primary)'}
                                onMouseLeave={(e) => e.currentTarget.style.color = 'var(--ifm-color-emphasis-600)'}
                            >
                                {showPassword ? <FaEyeSlash /> : <FaEye />}
                            </button>
                        </div>

                        {error && (
                            <div style={{
                                color: '#ff4d4d',
                                textAlign: 'center',
                                fontSize: '0.9rem',
                                padding: '0.75rem',
                                backgroundColor: 'rgba(255, 77, 77, 0.1)',
                                borderRadius: '8px',
                                marginBottom: '1rem',
                                border: '1px solid rgba(255, 77, 77, 0.3)'
                            }}>
                                <strong>⚠️ Error:</strong><br />
                                {error}
                                {error.includes('already registered') && (
                                    <div style={{ marginTop: '0.5rem', fontSize: '0.85rem' }}>
                                        <span
                                            onClick={toggleMode}
                                            style={{
                                                color: '#00d4ff',
                                                cursor: 'pointer',
                                                textDecoration: 'underline'
                                            }}
                                        >
                                            Click here to Sign In instead
                                        </span>
                                    </div>
                                )}
                            </div>
                        )}

                        {success && (
                            <div style={{
                                color: '#00d4ff',
                                textAlign: 'center',
                                fontSize: '0.9rem',
                                padding: '0.75rem',
                                backgroundColor: 'rgba(0, 212, 255, 0.1)',
                                borderRadius: '8px',
                                marginBottom: '1rem',
                                border: '1px solid rgba(0, 212, 255, 0.3)'
                            }}>
                                <strong>✅ Success!</strong><br />
                                {success}
                            </div>
                        )}

                        <button type="submit" className={styles.submitButton} disabled={loading}>
                            {loading ? (
                                <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
                                    <span className={styles.spinner}></span>
                                    Processing...
                                </span>
                            ) : (
                                isLogin ? 'Sign In' : 'Sign Up'
                            )}
                        </button>
                    </form>

                    <div className={styles.footer}>
                        <p>
                            {isLogin ? "Don't have an account?" : "Already have an account?"}
                            <span
                                className={styles.link}
                                onClick={toggleMode}
                            >
                                {isLogin ? ' Sign Up' : ' Sign In'}
                            </span>
                        </p>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default AuthModal;
